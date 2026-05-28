#include <chrono>
#include <memory>
#include <iostream>
#include <functional>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <nlohmann/json.hpp>
#include <vector>
#include <iomanip>
#include <sstream>
#include <string>
#include <array>
#include "xr_msgs/msg/custom.hpp"
#include "xr_msgs/msg/head.hpp"
#include "xr_msgs/msg/controller.hpp"

#include "PXREARobotSDK.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"

#include <filesystem>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "arm_interfaces/msg/master_controller_command.hpp"
#include "arm_interfaces/msg/arm_status.hpp"

// 添加 Pinocchio 和 CasADi 相关头文件
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/log.hpp>
#include <casadi/casadi.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using json = nlohmann::json;

// IK 求解器类
class IKSolver {
public:
    IKSolver(const std::string& urdf_path, const std::string& ee_frame_name)
        : urdf_path_(urdf_path), ee_frame_name_(ee_frame_name) {
        initialize();
    }
    
    bool solve(const Eigen::Matrix4d& target_pose, 
               const Eigen::VectorXd& q_guess,
               Eigen::VectorXd& q_solution,
               const Eigen::VectorXd& q_last = Eigen::VectorXd()) {
        
        // 设置目标位姿
        casadi::DM Tf_dm = matrix_to_dm(target_pose);
        opti_.set_value(param_tf_, Tf_dm);
        
        // 设置初始猜测
        casadi::DM q_init = eigen_to_dm(q_guess);
        opti_.set_initial(var_q_, q_init);
        
        // 设置上一时刻关节角（用于平滑项）
        if (q_last.size() > 0) {
            casadi::DM q_last_dm = eigen_to_dm(q_last);
            opti_.set_value(var_q_last_, q_last_dm);
        } else {
            opti_.set_value(var_q_last_, q_init);
        }
        
        // 求解
        try {
            casadi::OptiSol sol = opti_.solve();
            casadi::DM q_sol_dm = sol.value(var_q_);
            q_solution = dm_to_eigen(q_sol_dm);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "IK solve error: " << e.what() << std::endl;
            return false;
        }
    }
    
private:
    void initialize() {
        // 1. 加载模型（使用 SX 进行符号计算）
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path_, model);
        
        // 2. 转换为 CasADi 符号模型（SX）
        using ADScalar = casadi::SX;
        pinocchio::ModelTpl<ADScalar> ad_model = model.cast<ADScalar>();
        pinocchio::DataTpl<ADScalar> ad_data(ad_model);
        
        // 3. 定义 CasADi 符号关节变量（SX）
        casadi::SX q_sym = casadi::SX::sym("q", model.nq);
        casadi::SX Tf_sym = casadi::SX::sym("tf", 4, 4);
        
        pinocchio::ModelTpl<ADScalar>::ConfigVectorType q_ad(model.nq);
        pinocchio::casadi::copy(q_sym, q_ad);
        
        // 4. 符号前向运动学
        pinocchio::forwardKinematics(ad_model, ad_data, q_ad);
        pinocchio::updateFramePlacements(ad_model, ad_data);
        
        // 获取末端 frame
        pinocchio::FrameIndex hand_id = ad_model.getFrameId(ee_frame_name_);
        
        // 5. 构建误差函数（SX）
        // 平移误差
        auto tcol = [](const casadi::SX& T4x4) {
            return T4x4(casadi::Slice(0,3), casadi::Slice(3,4));
        };
        
        auto eig3_to_sx = [](const Eigen::Matrix<casadi::SX,3,1>& v) {
            casadi::SX out(3,1);
            for(int i=0;i<3;++i) out(i) = v(i);
            return out;
        };
        
        casadi::SX p = eig3_to_sx(ad_data.oMf[hand_id].translation());
        casadi::SX p_err = p - tcol(Tf_sym);
        
        // 旋转误差（SO(3) 对数映射）
        const Eigen::Matrix<casadi::SX,3,3>& R = ad_data.oMf[hand_id].rotation();
        Eigen::Matrix<casadi::SX,3,3> R_des;
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                R_des(i,j) = Tf_sym(i,j);
            }
        }
        
        Eigen::Matrix<casadi::SX,3,3> Rerr = R_des.transpose() * R;
        Eigen::Matrix<casadi::SX,3,3> skew = 0.5 * (Rerr - Rerr.transpose());
        Eigen::Matrix<casadi::SX,3,1> rot_err;
        rot_err << skew(2,1), skew(0,2), skew(1,0);
        
        casadi::SX rot_err_sx = casadi::SX::vertcat({rot_err(0), rot_err(1), rot_err(2)});
        
        // 6. 创建 CasADi 函数（SX -> SX）
        casadi::Function translational_error_func("translational_error", 
            {q_sym, Tf_sym}, {p_err});
        casadi::Function rotational_error_func("rotational_error", 
            {q_sym, Tf_sym}, {rot_err_sx});
        
        // 7. 构建优化问题（使用 MX）
        opti_ = casadi::Opti();
        
        // 优化变量和参数（MX）
        var_q_ = opti_.variable(model.nq);
        var_q_last_ = opti_.parameter(model.nq);
        param_tf_ = opti_.parameter(4, 4);
        q_ref_ = opti_.parameter(model.nq);
        joint_w_ = opti_.parameter(model.nq);
        
        // 调用 SX 函数，但传入 MX 变量，CasADi 会自动转换
        casadi::MX translational_cost = casadi::MX::sumsqr(
            translational_error_func(casadi::MXVector{var_q_, param_tf_}).at(0));
        casadi::MX rotational_cost = casadi::MX::sumsqr(
            rotational_error_func(casadi::MXVector{var_q_, param_tf_}).at(0));
        
        // 其他代价项
        casadi::MX smooth_cost = casadi::MX::sumsqr(var_q_ - var_q_last_);
        casadi::MX joint_i_cost = casadi::MX::sumsqr(casadi::MX::diag(joint_w_) * (var_q_ - q_ref_));
        
        // 约束：关节限位
        Eigen::VectorXd q_min = model.lowerPositionLimit;
        Eigen::VectorXd q_max = model.upperPositionLimit;
        casadi::DM q_min_dm(q_min.size(), 1);
        casadi::DM q_max_dm(q_max.size(), 1);
        for (int i = 0; i < q_min.size(); ++i) {
            q_min_dm(i) = q_min(i);
            q_max_dm(i) = q_max(i);
        }
        opti_.subject_to(opti_.bounded(q_min_dm, var_q_, q_max_dm));
        
        // 总代价
        opti_.minimize(50.0 * translational_cost + rotational_cost + 0.04 * smooth_cost + 0.06 * joint_i_cost);
        
        // 设置参考值和权重（默认值）
        casadi::DM ref_data = casadi::DM::zeros(model.nq, 1);
        opti_.set_value(q_ref_, ref_data);
        
        casadi::DM w_data = casadi::DM::ones(model.nq, 1);
        opti_.set_value(joint_w_, w_data);
        
        // 求解器配置
        casadi::Dict opts;
        opts["expand"] = true;
        opts["detect_simple_bounds"] = true;
        opts["calc_lam_p"] = false;
        opts["print_time"] = false;
        opts["ipopt.sb"] = "yes";
        opts["ipopt.print_level"] = 0;
        opts["ipopt.max_iter"] = 30;
        opts["ipopt.tol"] = 1e-4;
        opts["ipopt.acceptable_tol"] = 5e-4;
        opts["ipopt.acceptable_iter"] = 5;
        opts["ipopt.max_wall_time"] = 1e-2;
        opts["ipopt.warm_start_init_point"] = "yes";
        opts["ipopt.derivative_test"] = "none";
        opts["ipopt.jacobian_approximation"] = "exact";
        
        opti_.solver("ipopt", opts);
    }
    
    // 辅助函数
    casadi::DM matrix_to_dm(const Eigen::Matrix4d& matrix) {
        casadi::DM ret = casadi::DM::eye(4);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                ret(i,j) = matrix(i,j);
            }
        }
        return ret;
    }
    
    casadi::DM eigen_to_dm(const Eigen::VectorXd& vec) {
        casadi::DM dm(vec.size(), 1);
        for(int i = 0; i < vec.size(); ++i) {
            dm(i) = vec(i);
        }
        return dm;
    }
    
    Eigen::VectorXd dm_to_eigen(const casadi::DM& dm) {
        Eigen::VectorXd vec(dm.size1());
        for(int i = 0; i < dm.size1(); ++i) {
            vec(i) = static_cast<double>(dm(i));
        }
        return vec;
    }
    
private:
    std::string urdf_path_;
    std::string ee_frame_name_;
    
    // CasADi 优化相关（MX 类型）
    casadi::Opti opti_;
    casadi::MX var_q_;
    casadi::MX var_q_last_;
    casadi::MX param_tf_;
    casadi::MX q_ref_;
    casadi::MX joint_w_;
};

// 辅助函数（原第一个模块中的）
std::vector<double> DMToStdVector(const casadi::DM& dm) {
    std::vector<double> v(dm.size1());
    for (int i = 0; i < dm.size1(); ++i)
        v[i] = static_cast<double>(dm(i));
    return v;
}

casadi::DM matrix_to_dm(const Eigen::Matrix4d& matrix) {
    casadi::DM ret = casadi::DM::eye(4);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            ret(i,j) = matrix(i,j);
        }
    }
    return ret;
}

std::function<void(void* context, PXREAClientCallbackType type, int status, void* userData)> g_callback;
std::mutex g_callback_mutex;
void callbackForwarder(void* context, PXREAClientCallbackType type, int status, void* userData) {
    std::lock_guard<std::mutex> lock(g_callback_mutex);
    if (g_callback) {
        g_callback(context, type, status, userData);
    }
}

void print_json(const json& j, int indent=1) {
    std::string indent_str(indent * 2, ' ');
    if (j.is_object()) {
        std::cout << indent_str << "{\n";
        for (auto it = j.begin(); it != j.end(); ++it) {
            std::cout << indent_str << "  \"" << it.key() << "\": ";
            print_json(it.value(), indent + 1);
            std::cout << ",\n";
        }
        if (!j.empty()) {
            std::cout << "\b\b" << std::endl;
        }
        std::cout << indent_str << "}";
    } else if (j.is_array()) {
        std::cout << indent_str << "[\n";
        for (const auto& item : j) {
            std::cout << indent_str << "  ";
            print_json(item, indent + 1);
            std::cout << ",\n";
        }
        if (!j.empty()) {
            std::cout << "\b\b" << std::endl;
        }
        std::cout << indent_str << "]";
    } else if (j.is_string()) {
        std::cout << "\"" << j.get<std::string>() << "\"";
    } else if (j.is_boolean()) {
        std::cout << (j.get<bool>() ? "true" : "false");
    } else if (j.is_number_integer()) {
        std::cout << j.get<int64_t>();
    } else if (j.is_number_unsigned()) {
        std::cout << j.get<uint64_t>();
    } else if (j.is_number_float()) {
        std::cout << j.get<double>();
    } else if (j.is_null()) {
        std::cout << "null";
    } else {
        std::cout << "unknown type";
    }
}

std::vector<float> stringToFloatVector(const std::string& input) {
    std::vector<float> result;
    std::stringstream ss(input);
    std::string token;
    while (std::getline(ss, token, ',')) {
        try {
            result.push_back(std::stof(token));
        } catch (const std::exception& e) {
            std::cerr << "转换错误: " << token << " -> " << e.what() << std::endl;
        }
    }
    return result;
}

class XRNode : public rclcpp::Node {
public:
    XRNode() : Node("pico") {
        publisher_ = this->create_publisher<xr_msgs::msg::Custom>("xr_pose", 10);

        l_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        l_gripper_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/left/control/joint_states", 10);
        r_joint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/right/control/move_p", 10);
        r_gripper_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/right/control/joint_states", 10);

        l_real_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/left/feedback/tcp_pose", 10, std::bind(&XRNode::lPoseCallback, this, std::placeholders::_1));
        r_real_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/right/feedback/tcp_pose", 10, std::bind(&XRNode::rPoseCallback, this, std::placeholders::_1));

        // 创建紧急停止服务客户端
        l_emergency_stop_client_ = this->create_client<std_srvs::srv::Empty>("/left/emergency_stop");
        r_emergency_stop_client_ = this->create_client<std_srvs::srv::Empty>("/right/emergency_stop");
        
        // 初始化 IK 求解器
        std::string urdf_path = "/projects/pico_tele_agx/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf";
        std::string ee_frame_name = "joint7";
        ik_solver_ = std::make_shared<IKSolver>(urdf_path, ee_frame_name);
        
        // 初始化上一时刻的关节角（用于平滑项）
        last_q_ = Eigen::VectorXd::Zero(7); // 假设7轴机械臂

        joint_msg_.header.frame_id = "base_link";
        joint_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        joint_msg_.position.resize(7);
        joint_msg_.velocity.clear(); // 明确置空
        joint_msg_.effort.clear();  // 明确置空
    }

    ~XRNode() {}

    void lPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (l_real_has_new_pose_ == false) {
            l_real_pose.pose.position.x = msg->pose.position.x;
            l_real_pose.pose.position.y = msg->pose.position.y;
            l_real_pose.pose.position.z = msg->pose.position.z;
            l_real_pose.pose.orientation.x = msg->pose.orientation.x;
            l_real_pose.pose.orientation.y = msg->pose.orientation.y;
            l_real_pose.pose.orientation.z = msg->pose.orientation.z;
            l_real_pose.pose.orientation.w = msg->pose.orientation.w;
            l_real_has_new_pose_ = true;

            RCLCPP_INFO(this->get_logger(), "\033[1;33mTCP位姿 topic: %s\n- 位置: [x: %.3f, y: %.3f, z: %.3f]\n姿态: [qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f]\033[0m", 
                        l_real_pose_subscriber_->get_topic_name(),
                        l_real_pose.pose.position.x, l_real_pose.pose.position.y, l_real_pose.pose.position.z,
                        l_real_pose.pose.orientation.x, l_real_pose.pose.orientation.y, 
                        l_real_pose.pose.orientation.z, l_real_pose.pose.orientation.w);
        }
    }

    void rPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (r_real_has_new_pose_ == false) {
            r_real_pose.pose.position.x = msg->pose.position.x;
            r_real_pose.pose.position.y = msg->pose.position.y;
            r_real_pose.pose.position.z = msg->pose.position.z;
            r_real_pose.pose.orientation.x = msg->pose.orientation.x;
            r_real_pose.pose.orientation.y = msg->pose.orientation.y;
            r_real_pose.pose.orientation.z = msg->pose.orientation.z;
            r_real_pose.pose.orientation.w = msg->pose.orientation.w;
            r_real_has_new_pose_ = true;

            RCLCPP_INFO(this->get_logger(), "\033[1;33mTCP位姿 topic: %s\n- 位置: [x: %.3f, y: %.3f, z: %.3f]\n姿态: [qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f]\033[0m", 
                        r_real_pose_subscriber_->get_topic_name(),
                        r_real_pose.pose.position.x, r_real_pose.pose.position.y, r_real_pose.pose.position.z,
                        r_real_pose.pose.orientation.x, r_real_pose.pose.orientation.y, 
                        r_real_pose.pose.orientation.z, r_real_pose.pose.orientation.w);
        }
    }

    void publishJointState(const Eigen::VectorXd& q) {
        joint_msg_.header.stamp = this->now();
        for (int i = 0; i < 7; ++i) {
            joint_msg_.position[i] = q(i);
        }
        l_joint_publisher_->publish(joint_msg_);
    }

    void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData) {
        (void)context;
        switch (type) {
        case PXREAServerConnect:
            std::cout << "server connect" << std::endl;
            break;
        case PXREAServerDisconnect:
            std::cout << "server disconnect" << std::endl;
            break;
        case PXREADeviceFind:
            std::cout << "device find" << (const char*)userData << std::endl;
            break;
        case PXREADeviceMissing:
            std::cout << "device missing" << (const char*)userData << std::endl;
            break;
        case PXREADeviceConnect:
            std::cout << "device connect" << (const char*)userData << status << std::endl;
            break;
        case PXREADeviceStateJson: {
            auto& dsj = *((PXREADevStateJson*)userData);
            try {
                auto json_obj = json::parse(dsj.stateJson);
                auto value_str = json_obj["value"].get<std::string>();
                auto value_obj = json::parse(value_str);

                auto custom_msg = xr_msgs::msg::Custom();
                custom_msg.timestamp_ns = value_obj["timeStampNs"].get<uint64_t>();
                custom_msg.input = value_obj["Input"].get<int>();

                // head
                auto head_msg = xr_msgs::msg::Head();
                if (value_obj.contains("Head")) {
                    auto head_j = value_obj["Head"];
                    std::vector<float> head_pose = stringToFloatVector(head_j["pose"].get<std::string>());
                    if (head_pose.size() != 7) {
                        std::cerr << "Parse failed: head pose data length != 7" << std::endl;
                    }
                    std::copy(head_pose.begin(), head_pose.end(), head_msg.pose.data());
                    head_msg.status = head_j["status"].get<int>();
                } else {
                    head_msg.status = -1;
                }
                custom_msg.head = head_msg;

                // controller
                if (value_obj.contains("Controller")) {
                    for (auto& element : value_obj["Controller"].items()) {
                        auto controller_msg = xr_msgs::msg::Controller();
                        auto ctrl_j = element.value();

                        controller_msg.axis_x = ctrl_j["axisX"].get<float>();
                        controller_msg.axis_y = ctrl_j["axisY"].get<float>();
                        controller_msg.axis_click = ctrl_j["axisClick"].get<bool>();
                        controller_msg.gripper = ctrl_j["grip"].get<float>();
                        controller_msg.trigger = ctrl_j["trigger"].get<float>();
                        controller_msg.primary_button = ctrl_j["primaryButton"].get<bool>();
                        controller_msg.secondary_button = ctrl_j["secondaryButton"].get<bool>();
                        controller_msg.menu_button = ctrl_j["menuButton"].get<bool>();
                        std::vector<float> ctrl_pose = stringToFloatVector(ctrl_j["pose"].get<std::string>());
                        if (ctrl_pose.size() != 7) {
                            std::cerr << "Parse failed: ctrl pose data length != 7" << std::endl;
                        }
                        std::copy(ctrl_pose.begin(), ctrl_pose.end(), controller_msg.pose.data());
                        controller_msg.status = 3;

                        if (element.key() == "left") {
                            custom_msg.left_controller = controller_msg;
                        } else {
                            custom_msg.right_controller = controller_msg;
                        }
                    }
                } else {
                    auto left_controller_msg = xr_msgs::msg::Controller();
                    auto right_controller_msg = xr_msgs::msg::Controller();
                    left_controller_msg.status = -1;
                    right_controller_msg.status = -1;
                    custom_msg.left_controller = left_controller_msg;
                    custom_msg.right_controller = right_controller_msg;
                }

                publisher_->publish(custom_msg);

                if (custom_msg.left_controller.gripper == 1.0f) {
                    geometry_msgs::msg::PoseStamped ps;
                    ps.header.stamp = this->now();
                    ps.header.frame_id = "Pico";

                    ps.pose.position.x = custom_msg.left_controller.pose[0];
                    ps.pose.position.y = custom_msg.left_controller.pose[1];
                    ps.pose.position.z = custom_msg.left_controller.pose[2];
                    ps.pose.orientation.x = custom_msg.left_controller.pose[3];
                    ps.pose.orientation.y = custom_msg.left_controller.pose[4];
                    ps.pose.orientation.z = custom_msg.left_controller.pose[5];
                    ps.pose.orientation.w = custom_msg.left_controller.pose[6];

                    if (l_ctl_init == false) {
                        // 更新真机姿态
                        l_real_has_new_pose_ = false;
                        l_ctl_init_pose.header.stamp = ps.header.stamp;
                        l_ctl_init_pose.header.frame_id = ps.header.frame_id;
                        l_ctl_init_pose.pose.position.x = ps.pose.position.x;
                        l_ctl_init_pose.pose.position.y = ps.pose.position.y;
                        l_ctl_init_pose.pose.position.z = ps.pose.position.z;
                        l_ctl_init_pose.pose.orientation.x = ps.pose.orientation.x;
                        l_ctl_init_pose.pose.orientation.y = ps.pose.orientation.y;
                        l_ctl_init_pose.pose.orientation.z = ps.pose.orientation.z;
                        l_ctl_init_pose.pose.orientation.w = ps.pose.orientation.w;
                        l_ctl_init = true;

                        RCLCPP_INFO(this->get_logger(), 
                            "\033[1;33m[LEFT CTRL INIT] Pose: [x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f]\033[0m",
                            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
                            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
                            ps.pose.orientation.w);
                    }
                    else if (l_ctl_init == true && l_real_has_new_pose_ == true) {

                        // 计算控制器相对运动并转换到真实世界坐标系
                        double dx_ctl = ps.pose.position.x - l_ctl_init_pose.pose.position.x;
                        double dy_ctl = ps.pose.position.y - l_ctl_init_pose.pose.position.y;
                        double dz_ctl = ps.pose.position.z - l_ctl_init_pose.pose.position.z;
                        
                        double dx_real = -dz_ctl;
                        double dy_real = -dx_ctl;
                        double dz_real = dy_ctl;
                        
                        tf2::Quaternion q_ctl_init(
                            l_ctl_init_pose.pose.orientation.x,
                            l_ctl_init_pose.pose.orientation.y,
                            l_ctl_init_pose.pose.orientation.z,
                            l_ctl_init_pose.pose.orientation.w);
                        
                        tf2::Quaternion q_ctl_current(
                            ps.pose.orientation.x,
                            ps.pose.orientation.y,
                            ps.pose.orientation.z,
                            ps.pose.orientation.w);
                        
                        tf2::Quaternion q_ctl_rel = q_ctl_init.inverse() * q_ctl_current;
                        q_ctl_rel.normalize();

                        tf2::Matrix3x3 R_ctl_to_real(0, 0, -1, -1, 0, 0, 0, 1, 0);
                        tf2::Matrix3x3 R_ctl_rel(q_ctl_rel);
                        tf2::Matrix3x3 R_real_rel = R_ctl_to_real * R_ctl_rel * R_ctl_to_real.transpose();

                        tf2::Quaternion q_real_rel;
                        R_real_rel.getRotation(q_real_rel);
                        q_real_rel.normalize();

                        geometry_msgs::msg::PoseStamped result;
                        result.header.stamp = this->now();
                        result.header.frame_id = "real";

                        result.pose.position.x = l_real_pose.pose.position.x + dx_real;
                        result.pose.position.y = l_real_pose.pose.position.y + dy_real;
                        result.pose.position.z = l_real_pose.pose.position.z + dz_real;

                        tf2::Quaternion q_real_current(
                            l_real_pose.pose.orientation.x,
                            l_real_pose.pose.orientation.y,
                            l_real_pose.pose.orientation.z,
                            l_real_pose.pose.orientation.w);

                        tf2::Quaternion q_real_new = q_real_rel * q_real_current;
                        q_real_new.normalize();

                        result.pose.orientation.x = q_real_new.x();
                        result.pose.orientation.y = q_real_new.y();
                        result.pose.orientation.z = q_real_new.z();
                        result.pose.orientation.w = q_real_new.w();

                        // ========== 在这里调用 IK 求解器 ==========
                        // 将 result 转换为 Eigen::Matrix4d
                        Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
                        target_pose(0,3) = result.pose.position.x;
                        target_pose(1,3) = result.pose.position.y;
                        target_pose(2,3) = result.pose.position.z;
                        
                        // 将四元数转换为旋转矩阵
                        tf2::Quaternion q_result(
                            result.pose.orientation.x,
                            result.pose.orientation.y,
                            result.pose.orientation.z,
                            result.pose.orientation.w);
                        tf2::Matrix3x3 rot_mat(q_result);
                        for (int i = 0; i < 3; ++i) {
                            for (int j = 0; j < 3; ++j) {
                                target_pose(i,j) = rot_mat[i][j];
                            }
                        }
                        
                        // 初始猜测：使用上一时刻的解
                        Eigen::VectorXd q_guess = last_q_;
                        
                        // 求解逆解
                        Eigen::VectorXd q_solution;
                        bool ik_success = ik_solver_->solve(target_pose, q_guess, q_solution, last_q_);
                        
                        if (ik_success) {
                            // 更新上一时刻的解
                            last_q_ = q_solution;
                            
                            // 您可能需要一个发布关节角的 publisher，这里假设为 l_joint_state_publisher_
                            // l_joint_state_publisher_->publish(joint_msg);
                            
                            RCLCPP_INFO(this->get_logger(), 
                                "\033[1;32mtopic: %s\nIK solution: [%f, %f, %f, %f, %f, %f, %f]\033[0m",
                                l_joint_publisher_->get_topic_name(),
                                q_solution(0), q_solution(1), q_solution(2), q_solution(3),
                                q_solution(4), q_solution(5), q_solution(6));
                            
                            publishJointState(q_solution);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "\033[1;31mIK solve failed!\033[0m");
                            // IK 失败时，可以选择发布上一个成功的解或者不发布
                        }
                        // ========== IK 调用结束 ==========
                    }
                }
                else {
                    if (l_ctl_init) {
                        l_ctl_init = false;
                        RCLCPP_INFO(this->get_logger(), "\033[1;31mLeft trigger released, calling emergency stop service\033[0m");
                        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                        l_emergency_stop_client_->async_send_request(request);
                        RCLCPP_INFO(this->get_logger(), "\033[1;31mEmergency stop service called\033[0m");
                    }
                }

                if (custom_msg.left_controller.trigger == 1.0f && left_gripper == false) {
                    auto message = sensor_msgs::msg::JointState();
                    message.header.stamp = this->now();
                    message.header.frame_id = "base_link";
                    
                    float gripper_value = 0.07f;
                    message.name = {"gripper"};
                    message.position = {gripper_value};
                    message.velocity = {0.0};
                    message.effort = {1.0};
                    l_gripper_joint_publisher_->publish(message);
                    left_gripper = true;

                    RCLCPP_INFO(this->get_logger(), "Left gripper: [%f]", gripper_value);
                }
                else if (custom_msg.left_controller.trigger != 1.0f && left_gripper == true) {
                    auto message = sensor_msgs::msg::JointState();
                    message.header.stamp = this->now();
                    message.header.frame_id = "base_link";
                    
                    float gripper_value = 0.0f;
                    message.name = {"gripper"};
                    message.position = {gripper_value};
                    message.velocity = {0.0};
                    message.effort = {1.0};
                    l_gripper_joint_publisher_->publish(message);
                    left_gripper = false;

                    RCLCPP_INFO(this->get_logger(), "Left gripper: [%f]", gripper_value);
                }

            } catch (const std::exception& e) {
                std::cerr << "Parse failed: " << e.what() << std::endl;
            }
            break;
        }
        case PXREADeviceCustomMessage:
            std::cout << "device custom message" << std::endl;
            break;
        case PXREAFullMask:
            std::cout << "full mask" << std::endl;
            break;
        }
    }

private:
    rclcpp::Publisher<xr_msgs::msg::Custom>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr l_joint_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r_joint_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr l_gripper_joint_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr r_gripper_joint_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr l_real_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr r_real_pose_subscriber_;
    bool l_real_has_new_pose_ = false;
    bool r_real_has_new_pose_ = false;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr l_emergency_stop_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr r_emergency_stop_client_;

    bool left_gripper = false;
    bool right_gripper = false;

    bool l_ctl_init = false;
    bool r_ctl_init = false;
    geometry_msgs::msg::PoseStamped l_ctl_init_pose;
    geometry_msgs::msg::PoseStamped l_real_pose;
    geometry_msgs::msg::PoseStamped r_ctl_init_pose;
    geometry_msgs::msg::PoseStamped r_real_pose;
    
    // IK 求解器相关
    std::shared_ptr<IKSolver> ik_solver_;
    Eigen::VectorXd last_q_;  // 上一时刻的关节角，用于平滑项
    sensor_msgs::msg::JointState joint_msg_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto xrNode = std::make_shared<XRNode>();
    g_callback = [&xrNode] (void* context, PXREAClientCallbackType type, int status, void* userData) {
        xrNode->OnPXREAClientCallback(context, type, status, userData);
    };
    PXREAInit(NULL, callbackForwarder, PXREAFullMask);

    rclcpp::spin(xrNode);

    PXREADeinit();

    rclcpp::shutdown();
    return 0;
}