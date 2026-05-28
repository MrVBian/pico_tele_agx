#ifndef MAIN_H
#define MAIN_H

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

// Pinocchio 和 CasADi 相关头文件
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/log.hpp>
#include <casadi/casadi.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using json = nlohmann::json;

// IK 求解器类声明
class IKSolver {
public:
    IKSolver(const std::string& urdf_path, const std::string& ee_frame_name);
    
    bool solve(const Eigen::Matrix4d& target_pose, 
               const Eigen::VectorXd& q_guess,
               Eigen::VectorXd& q_solution,
               const Eigen::VectorXd& q_last = Eigen::VectorXd());
    
private:
    void initialize();
    
    // 辅助函数声明
    casadi::DM matrix_to_dm(const Eigen::Matrix4d& matrix);
    casadi::DM eigen_to_dm(const Eigen::VectorXd& vec);
    Eigen::VectorXd dm_to_eigen(const casadi::DM& dm);
    
private:
    std::string urdf_path_;
    std::string ee_frame_name_;
    
    // CasADi 优化相关
    casadi::Opti opti_;
    casadi::MX var_q_;
    casadi::MX var_q_last_;
    casadi::MX param_tf_;
    casadi::MX q_ref_;
    casadi::MX joint_w_;
};

// 辅助函数声明
std::vector<double> DMToStdVector(const casadi::DM& dm);
casadi::DM matrix_to_dm(const Eigen::Matrix4d& matrix);
std::function<void(void* context, PXREAClientCallbackType type, int status, void* userData)> g_callback;
std::mutex g_callback_mutex;
void callbackForwarder(void* context, PXREAClientCallbackType type, int status, void* userData);
void print_json(const json& j, int indent=1);
std::vector<float> stringToFloatVector(const std::string& input);

// XRNode 类声明
class XRNode : public rclcpp::Node {
public:
    XRNode();
    ~XRNode();

    void lPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishJointState(const Eigen::VectorXd& q);
    void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData);

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
    Eigen::VectorXd last_q_;
    sensor_msgs::msg::JointState joint_msg_;
};

#endif // MAIN_H