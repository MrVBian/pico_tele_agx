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
#include "sensor_msgs/msg/joint_state.hpp"  // 添加 JointState 头文件

#include <filesystem>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using json = nlohmann::json;

std::function<void(void* context, PXREAClientCallbackType type, int status, void* userData)> g_callback;

std::mutex g_callback_mutex;

void callbackForwarder(void* context, PXREAClientCallbackType type, int status, void* userData) {
  std::lock_guard<std::mutex> lock(g_callback_mutex);
  if (g_callback) {
    g_callback(context, type, status, userData);
  }
}

void print_json(const json& j, int indent=1) {
    // 根据缩进级别设置空格
    std::string indent_str(indent * 2, ' ');

    if (j.is_object()) {
        std::cout << indent_str << "{\n";
        for (auto it = j.begin(); it != j.end(); ++it) {
            std::cout << indent_str << "  \"" << it.key() << "\": ";
            print_json(it.value(), indent + 1);
            std::cout << ",\n";
        }
        // 删除最后一个多余的逗号
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

class XRNode : public rclcpp::Node
{
public:
  XRNode() : Node("xr_publisher")
  {
    publisher_ = this->create_publisher<xr_msgs::msg::Custom>("xr_pose", 10);
    
    // sim
    // l_joint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/xr/move_p", 10);
    
    
    // real
    // l_joint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/control/move_p", 10);
    // l_gripper_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/control/joint_states", 10);
    l_joint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/left/control/move_p", 10);
    l_gripper_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/left/control/joint_states", 10);
    r_joint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/right/control/move_p", 10);
    r_gripper_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/right/control/joint_states", 10);

    l_real_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/left/feedback/tcp_pose", 10, std::bind(&XRNode::lPoseCallback, this, std::placeholders::_1));
    r_real_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/right/feedback/tcp_pose", 10, std::bind(&XRNode::rPoseCallback, this, std::placeholders::_1));
    void lPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // // 创建紧急停止服务客户端
    // l_emergency_stop_client_ = this->create_client<std_srvs::srv::Empty>("/left/emergency_stop");
    // r_emergency_stop_client_ = this->create_client<std_srvs::srv::Empty>("/right/emergency_stop");
  }

  ~XRNode() {
  }

  void lPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (l_real_has_new_pose_ == false){
      l_real_pose.pose.position.x = msg->pose.position.x;
      l_real_pose.pose.position.y = msg->pose.position.y;
      l_real_pose.pose.position.z = msg->pose.position.z;
      l_real_pose.pose.orientation.x = msg->pose.orientation.x;
      l_real_pose.pose.orientation.y = msg->pose.orientation.y;
      l_real_pose.pose.orientation.z = msg->pose.orientation.z;
      l_real_pose.pose.orientation.w = msg->pose.orientation.w;
      l_real_has_new_pose_ = true;

      // 打印接收到的信息
      RCLCPP_INFO(this->get_logger(), "\033[1;33mTCP位姿 topic: %s\n- 位置: [x: %.3f, y: %.3f, z: %.3f]\n姿态: [qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f]\033[0m", 
                  l_real_pose_subscriber_->get_topic_name(),
                  l_real_pose.pose.position.x, l_real_pose.pose.position.y, l_real_pose.pose.position.z,
                  l_real_pose.pose.orientation.x, l_real_pose.pose.orientation.y, 
                  l_real_pose.pose.orientation.z, l_real_pose.pose.orientation.w
      );
    }
  }

  void rPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (r_real_has_new_pose_ == false){
      r_real_pose.pose.position.x = msg->pose.position.x;
      r_real_pose.pose.position.y = msg->pose.position.y;
      r_real_pose.pose.position.z = msg->pose.position.z;
      r_real_pose.pose.orientation.x = msg->pose.orientation.x;
      r_real_pose.pose.orientation.y = msg->pose.orientation.y;
      r_real_pose.pose.orientation.z = msg->pose.orientation.z;
      r_real_pose.pose.orientation.w = msg->pose.orientation.w;
      r_real_has_new_pose_ = true;

      // 打印接收到的信息
      RCLCPP_INFO(this->get_logger(), "\033[1;33mTCP位姿 topic: %s\n- 位置: [x: %.3f, y: %.3f, z: %.3f]\n姿态: [qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f]\033[0m", 
                  r_real_pose_subscriber_->get_topic_name(),
                  r_real_pose.pose.position.x, r_real_pose.pose.position.y, r_real_pose.pose.position.z,
                  r_real_pose.pose.orientation.x, r_real_pose.pose.orientation.y, 
                  r_real_pose.pose.orientation.z, r_real_pose.pose.orientation.w
      );
    }
  }

  // 修改回调部分：不直接求解 IK，只更新 latest target 并 notify 工作线程
  void OnPXREAClientCallback(void* context, PXREAClientCallbackType type,int status,void* userData)
  {
    (void)context;
    switch (type)
    {
      case PXREAServerConnect:
          std::cout <<"server connect"  << std::endl;
          break;
      case PXREAServerDisconnect:
          std::cout  <<"server disconnect"  << std::endl;
          break;
      case PXREADeviceFind:
          std::cout << "device find"<< (const char*)userData << std::endl;
          break;
      case PXREADeviceMissing:
          std::cout <<"device missing"<<(const char*)userData<<  std::endl;
          break;
      case PXREADeviceConnect:
          std::cout <<"device connect"<<(const char*)userData<<status<< std::endl;
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

            // // XR
            publisher_->publish(custom_msg);

            if (custom_msg.left_controller.trigger == 1.0f) {
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
                  ps.pose.orientation.w
                );
              }
              else if(l_ctl_init == true && l_real_has_new_pose_ == true) {

                // 把控制器(右手系x右，y上，z后)里的移动和旋转，"翻译"成真实世界(右手系x前，y左，z上)里的动作
                // 然后叠加到当前的物体位置上，最后发布这个结果result
                
                // 1. 计算控制器相对于初始位置的位移
                double dx_ctl = ps.pose.position.x - l_ctl_init_pose.pose.position.x;
                double dy_ctl = ps.pose.position.y - l_ctl_init_pose.pose.position.y;
                double dz_ctl = ps.pose.position.z - l_ctl_init_pose.pose.position.z;
                
                // 2. 将控制器位移转换到真实世界坐标系
                // 控制器x+ -> 真实世界y-
                // 控制器y+ -> 真实世界z+
                // 控制器z+ -> 真实世界x-
                double dx_real = -dz_ctl;  // 控制器z+ -> 真实世界x-
                double dy_real = -dx_ctl;  // 控制器x+ -> 真实世界y-
                double dz_real = dy_ctl;   // 控制器y+ -> 真实世界z+
                
                // 3. 计算旋转变换
                // 获取控制器初始和当前的四元数
                tf2::Quaternion q_ctl_init(
                  l_ctl_init_pose.pose.orientation.x,
                  l_ctl_init_pose.pose.orientation.y,
                  l_ctl_init_pose.pose.orientation.z,
                  l_ctl_init_pose.pose.orientation.w
                );
                
                tf2::Quaternion q_ctl_current(
                  ps.pose.orientation.x,
                  ps.pose.orientation.y,
                  ps.pose.orientation.z,
                  ps.pose.orientation.w
                );
                















                // 计算相对旋转：从初始到当前的旋转
                tf2::Quaternion q_ctl_rel = q_ctl_init.inverse() * q_ctl_current;
                q_ctl_rel.normalize();

                // 4. 将控制器坐标系的旋转转换到真实世界坐标系
                // 使用旋转矩阵转换而不是欧拉角转换，避免万向节锁和顺序问题
                // 定义从控制器坐标系到真实世界坐标系的旋转矩阵
                // 根据坐标轴映射关系：控制器(x右,y上,z后) -> 真实世界(x前,y左,z上)
                // 控制器x轴(右) -> 真实世界y轴负方向(左)
                // 控制器y轴(上) -> 真实世界z轴(上)
                // 控制器z轴(后) -> 真实世界x轴负方向(后)
                tf2::Matrix3x3 R_ctl_to_real(
                  0, 0, -1,  // 控制器x轴(1,0,0)映射到真实世界(0,0,-1) -> 但实际上是y-，见下面
                  -1, 0, 0,  // 控制器y轴(0,1,0)映射到真实世界(-1,0,0) -> 但实际上是z+，见下面
                  0, 1, 0   // 控制器z轴(0,0,1)映射到真实世界(0,1,0) -> 但实际上是x-，见下面
                );
                // 修正：上面的矩阵不对，我们需要仔细映射
                // 实际上，控制器的基向量在真实世界中的表示为：
                // 控制器x(1,0,0) -> 真实世界(0,-1,0) 即y-
                // 控制器y(0,1,0) -> 真实世界(0,0,1) 即z+
                // 控制器z(0,0,1) -> 真实世界(-1,0,0) 即x-
                // 所以正确的旋转矩阵R_ctl_to_real为：
                // [ 0,  0, -1 ]
                // [-1,  0,  0 ]
                // [ 0,  1,  0 ]

                // 将控制器的相对旋转转换为旋转矩阵
                tf2::Matrix3x3 R_ctl_rel(q_ctl_rel);

                // 将控制器坐标系中的旋转矩阵转换到真实世界坐标系
                // 公式: R_real_rel = R_ctl_to_real * R_ctl_rel * R_ctl_to_real.transpose()
                tf2::Matrix3x3 R_real_rel = R_ctl_to_real * R_ctl_rel * R_ctl_to_real.transpose();

                // 将旋转矩阵转换回四元数
                tf2::Quaternion q_real_rel;
                R_real_rel.getRotation(q_real_rel);
                q_real_rel.normalize();

                // 5. 叠加到当前的物体位置
                geometry_msgs::msg::PoseStamped result;
                result.header.stamp = this->now();
                result.header.frame_id = "real";

                // 计算新的位置
                result.pose.position.x = l_real_pose.pose.position.x + dx_real;
                result.pose.position.y = l_real_pose.pose.position.y + dy_real;
                result.pose.position.z = l_real_pose.pose.position.z + dz_real;

                // 计算新的方向
                tf2::Quaternion q_real_current(
                  l_real_pose.pose.orientation.x,
                  l_real_pose.pose.orientation.y,
                  l_real_pose.pose.orientation.z,
                  l_real_pose.pose.orientation.w
                );

                // 旋转叠加：在全局坐标系中应用相对旋转
                tf2::Quaternion q_real_new = q_real_rel * q_real_current;
                q_real_new.normalize();

                // 调试：检查旋转轴
                // 获取四元数的旋转轴和角度
                tf2::Vector3 axis_ctl = q_ctl_rel.getAxis();
                double angle_ctl = q_ctl_rel.getAngle();

                tf2::Vector3 axis_real = q_real_rel.getAxis();
                double angle_real = q_real_rel.getAngle();

                // 为了调试，也计算欧拉角
                double roll_ctl, pitch_ctl, yaw_ctl;
                tf2::Matrix3x3(q_ctl_rel).getRPY(roll_ctl, pitch_ctl, yaw_ctl);

                double roll_real, pitch_real, yaw_real;
                tf2::Matrix3x3(q_real_rel).getRPY(roll_real, pitch_real, yaw_real);

                RCLCPP_INFO(this->get_logger(),
                  "\033[1;33m[LEFT CTRL DEBUG]\033[0m\n"
                  "┌──────────────────────────────────┬─────────────────────────────────────────────────────────┐\n"
                  "│ Controller offset                │ dx:%9.6f  dy:%9.6f  dz:%9.6f │\n"
                  "│ Transformed to real              │ dx:%9.6f  dy:%9.6f  dz:%9.6f │\n"
                  "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                  "│ Controller RPY (rad)            │ roll:%7.4f pitch:%7.4f yaw:%7.4f │\n"
                  "│ Real world RPY (rad)            │ roll:%7.4f pitch:%7.4f yaw:%7.4f │\n"
                  "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                  "│ Controller rel rotation          │ axis:[%6.3f, %6.3f, %6.3f] angle:%6.3f │\n"
                  "│ Real world rel rotation          │ axis:[%6.3f, %6.3f, %6.3f] angle:%6.3f │\n"
                  "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                  "│ Real world current pos           │ x:%9.6f  y:%9.6f  z:%9.6f │\n"
                  "│ Real world new pos               │ x:%9.6f  y:%9.6f  z:%9.6f │\n"
                  "│ Real world new quat              │ x:%9.6f  y:%9.6f  z:%9.6f  w:%9.6f │\n"
                  "└──────────────────────────────────┴─────────────────────────────────────────────────────────┘",
                  dx_ctl, dy_ctl, dz_ctl,
                  dx_real, dy_real, dz_real,
                  roll_ctl, pitch_ctl, yaw_ctl,
                  roll_real, pitch_real, yaw_real,
                  axis_ctl.x(), axis_ctl.y(), axis_ctl.z(), angle_ctl,
                  axis_real.x(), axis_real.y(), axis_real.z(), angle_real,
                  l_real_pose.pose.position.x, l_real_pose.pose.position.y, l_real_pose.pose.position.z,
                  result.pose.position.x, result.pose.position.y, result.pose.position.z,
                  q_real_new.x(), q_real_new.y(), q_real_new.z(), q_real_new.w()
                );

                result.pose.orientation.x = q_real_new.x();
                result.pose.orientation.y = q_real_new.y();
                result.pose.orientation.z = q_real_new.z();
                result.pose.orientation.w = q_real_new.w();

                l_joint_publisher_->publish(result);
              }
            }
            else {
              if (l_ctl_init) {
                l_ctl_init = false;
                // RCLCPP_INFO(this->get_logger(), "\033[1;31mLeft trigger released, calling emergency stop service\033[0m");
                // auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                // l_emergency_stop_client_->async_send_request(request);
                // RCLCPP_INFO(this->get_logger(), "\033[1;31mEmergency stop service called\033[0m");
              }
            }

            if (custom_msg.left_controller.gripper == 1.0f && left_gripper == false) {
              auto message = sensor_msgs::msg::JointState();
              message.header.stamp = this->now();
              message.header.frame_id = "base_link";
              
              float gripper_value = 0.05f;
              message.name = {"gripper"};
              message.position = {gripper_value};
              message.velocity = {0.0};  // 使用空数组会导致错误，这里设为0.0
              message.effort = {1.0};
              l_gripper_joint_publisher_->publish(message);
              left_gripper = true;

              RCLCPP_INFO(this->get_logger(), "Left gripper: [%f]", gripper_value);
            }
            else if (custom_msg.left_controller.gripper != 1.0f && left_gripper == true){
              auto message = sensor_msgs::msg::JointState();
              message.header.stamp = this->now();
              message.header.frame_id = "base_link";
              
              float gripper_value = 0.0f;
              message.name = {"gripper"};
              message.position = {gripper_value};
              message.velocity = {0.0};  // 使用空数组会导致错误，这里设为0.0
              message.effort = {1.0};
              l_gripper_joint_publisher_->publish(message);
              left_gripper = false;

              RCLCPP_INFO(this->get_logger(), "Left gripper: [%f]", gripper_value);
            }




            if (custom_msg.right_controller.trigger == 1.0f) {
              geometry_msgs::msg::PoseStamped ps;
              ps.header.stamp = this->now();
              ps.header.frame_id = "Pico";

              ps.pose.position.x = custom_msg.right_controller.pose[0];
              ps.pose.position.y = custom_msg.right_controller.pose[1];
              ps.pose.position.z = custom_msg.right_controller.pose[2];
              ps.pose.orientation.x = custom_msg.right_controller.pose[3];
              ps.pose.orientation.y = custom_msg.right_controller.pose[4];
              ps.pose.orientation.z = custom_msg.right_controller.pose[5];
              ps.pose.orientation.w = custom_msg.right_controller.pose[6];

              if (r_ctl_init == false) {
                // 更新真机姿态
                r_real_has_new_pose_ = false;
                r_ctl_init_pose.header.stamp = ps.header.stamp;
                r_ctl_init_pose.header.frame_id = ps.header.frame_id;
                r_ctl_init_pose.pose.position.x = ps.pose.position.x;
                r_ctl_init_pose.pose.position.y = ps.pose.position.y;
                r_ctl_init_pose.pose.position.z = ps.pose.position.z;
                r_ctl_init_pose.pose.orientation.x = ps.pose.orientation.x;
                r_ctl_init_pose.pose.orientation.y = ps.pose.orientation.y;
                r_ctl_init_pose.pose.orientation.z = ps.pose.orientation.z;
                r_ctl_init_pose.pose.orientation.w = ps.pose.orientation.w;
                r_ctl_init = true;

                RCLCPP_INFO(this->get_logger(), 
                  "\033[1;33m[RIGHT CTRL INIT] Pose: [x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f]\033[0m",
                  ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
                  ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
                  ps.pose.orientation.w
                );
              }
              else if(r_ctl_init == true && r_real_has_new_pose_ == true) {

                // 把控制器(右手系x右，y上，z后)里的移动和旋转，"翻译"成真实世界(右手系x前，y左，z上)里的动作
                // 然后叠加到当前的物体位置上，最后发布这个结果result
                
                // 1. 计算控制器相对于初始位置的位移
                double dx_ctl = ps.pose.position.x - r_ctl_init_pose.pose.position.x;
                double dy_ctl = ps.pose.position.y - r_ctl_init_pose.pose.position.y;
                double dz_ctl = ps.pose.position.z - r_ctl_init_pose.pose.position.z;
                
                // 2. 将控制器位移转换到真实世界坐标系
                // 控制器x+ -> 真实世界y-
                // 控制器y+ -> 真实世界z+
                // 控制器z+ -> 真实世界x-
                double dx_real = -dz_ctl;  // 控制器z+ -> 真实世界x-
                double dy_real = -dx_ctl;  // 控制器x+ -> 真实世界y-
                double dz_real = dy_ctl;   // 控制器y+ -> 真实世界z+
                
                // 3. 计算旋转变换
                // 获取控制器初始和当前的四元数
                tf2::Quaternion q_ctl_init(
                  r_ctl_init_pose.pose.orientation.x,
                  r_ctl_init_pose.pose.orientation.y,
                  r_ctl_init_pose.pose.orientation.z,
                  r_ctl_init_pose.pose.orientation.w
                );
                
                tf2::Quaternion q_ctl_current(
                  ps.pose.orientation.x,
                  ps.pose.orientation.y,
                  ps.pose.orientation.z,
                  ps.pose.orientation.w
                );
                

                // 计算相对旋转：从初始到当前的旋转
                tf2::Quaternion q_ctl_rel = q_ctl_init.inverse() * q_ctl_current;
                q_ctl_rel.normalize();

                // 4. 将控制器坐标系的旋转转换到真实世界坐标系
                // 使用旋转矩阵转换而不是欧拉角转换，避免万向节锁和顺序问题
                // 定义从控制器坐标系到真实世界坐标系的旋转矩阵
                // 根据坐标轴映射关系：控制器(x右,y上,z后) -> 真实世界(x前,y左,z上)
                // 控制器x轴(右) -> 真实世界y轴负方向(左)
                // 控制器y轴(上) -> 真实世界z轴(上)
                // 控制器z轴(后) -> 真实世界x轴负方向(后)
                tf2::Matrix3x3 R_ctl_to_real(
                  0, 0, -1,  // 控制器x轴(1,0,0)映射到真实世界(0,0,-1) -> 但实际上是y-，见下面
                  -1, 0, 0,  // 控制器y轴(0,1,0)映射到真实世界(-1,0,0) -> 但实际上是z+，见下面
                  0, 1, 0   // 控制器z轴(0,0,1)映射到真实世界(0,1,0) -> 但实际上是x-，见下面
                );
                // 修正：上面的矩阵不对，我们需要仔细映射
                // 实际上，控制器的基向量在真实世界中的表示为：
                // 控制器x(1,0,0) -> 真实世界(0,-1,0) 即y-
                // 控制器y(0,1,0) -> 真实世界(0,0,1) 即z+
                // 控制器z(0,0,1) -> 真实世界(-1,0,0) 即x-
                // 所以正确的旋转矩阵R_ctl_to_real为：
                // [ 0,  0, -1 ]
                // [-1,  0,  0 ]
                // [ 0,  1,  0 ]

                // 将控制器的相对旋转转换为旋转矩阵
                tf2::Matrix3x3 R_ctl_rel(q_ctl_rel);

                // 将控制器坐标系中的旋转矩阵转换到真实世界坐标系
                // 公式: R_real_rel = R_ctl_to_real * R_ctl_rel * R_ctl_to_real.transpose()
                tf2::Matrix3x3 R_real_rel = R_ctl_to_real * R_ctl_rel * R_ctl_to_real.transpose();

                // 将旋转矩阵转换回四元数
                tf2::Quaternion q_real_rel;
                R_real_rel.getRotation(q_real_rel);
                q_real_rel.normalize();

                // 5. 叠加到当前的物体位置
                geometry_msgs::msg::PoseStamped result;
                result.header.stamp = this->now();
                result.header.frame_id = "real";

                // 计算新的位置
                result.pose.position.x = r_real_pose.pose.position.x + dx_real;
                result.pose.position.y = r_real_pose.pose.position.y + dy_real;
                result.pose.position.z = r_real_pose.pose.position.z + dz_real;

                // 计算新的方向
                tf2::Quaternion q_real_current(
                  r_real_pose.pose.orientation.x,
                  r_real_pose.pose.orientation.y,
                  r_real_pose.pose.orientation.z,
                  r_real_pose.pose.orientation.w
                );

                // 旋转叠加：在全局坐标系中应用相对旋转
                tf2::Quaternion q_real_new = q_real_rel * q_real_current;
                q_real_new.normalize();

                // 调试：检查旋转轴
                // 获取四元数的旋转轴和角度
                tf2::Vector3 axis_ctl = q_ctl_rel.getAxis();
                double angle_ctl = q_ctl_rel.getAngle();

                tf2::Vector3 axis_real = q_real_rel.getAxis();
                double angle_real = q_real_rel.getAngle();

                // 为了调试，也计算欧拉角
                double roll_ctl, pitch_ctl, yaw_ctl;
                tf2::Matrix3x3(q_ctl_rel).getRPY(roll_ctl, pitch_ctl, yaw_ctl);

                double roll_real, pitch_real, yaw_real;
                tf2::Matrix3x3(q_real_rel).getRPY(roll_real, pitch_real, yaw_real);

                RCLCPP_INFO(this->get_logger(),
                  "\033[1;33m[RIGHT CTRL DEBUG]\033[0m\n"
                  "┌──────────────────────────────────┬─────────────────────────────────────────────────────────┐\n"
                  "│ Controller offset                │ dx:%9.6f  dy:%9.6f  dz:%9.6f │\n"
                  "│ Transformed to real              │ dx:%9.6f  dy:%9.6f  dz:%9.6f │\n"
                  "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                  "│ Controller RPY (rad)            │ roll:%7.4f pitch:%7.4f yaw:%7.4f │\n"
                  "│ Real world RPY (rad)            │ roll:%7.4f pitch:%7.4f yaw:%7.4f │\n"
                  "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                  "│ Controller rel rotation          │ axis:[%6.3f, %6.3f, %6.3f] angle:%6.3f │\n"
                  "│ Real world rel rotation          │ axis:[%6.3f, %6.3f, %6.3f] angle:%6.3f │\n"
                  "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                  "│ Real world current pos           │ x:%9.6f  y:%9.6f  z:%9.6f │\n"
                  "│ Real world new pos               │ x:%9.6f  y:%9.6f  z:%9.6f │\n"
                  "│ Real world new quat              │ x:%9.6f  y:%9.6f  z:%9.6f  w:%9.6f │\n"
                  "└──────────────────────────────────┴─────────────────────────────────────────────────────────┘",
                  dx_ctl, dy_ctl, dz_ctl,
                  dx_real, dy_real, dz_real,
                  roll_ctl, pitch_ctl, yaw_ctl,
                  roll_real, pitch_real, yaw_real,
                  axis_ctl.x(), axis_ctl.y(), axis_ctl.z(), angle_ctl,
                  axis_real.x(), axis_real.y(), axis_real.z(), angle_real,
                  r_real_pose.pose.position.x, r_real_pose.pose.position.y, r_real_pose.pose.position.z,
                  result.pose.position.x, result.pose.position.y, result.pose.position.z,
                  q_real_new.x(), q_real_new.y(), q_real_new.z(), q_real_new.w()
                );

                result.pose.orientation.x = q_real_new.x();
                result.pose.orientation.y = q_real_new.y();
                result.pose.orientation.z = q_real_new.z();
                result.pose.orientation.w = q_real_new.w();

                r_joint_publisher_->publish(result);
              }
            }
            else {
              if (r_ctl_init) {
                r_ctl_init = false;
                // RCLCPP_INFO(this->get_logger(), "\033[1;31mRight trigger released, calling emergency stop service\033[0m");
                // auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                // r_emergency_stop_client_->async_send_request(request);
                // RCLCPP_INFO(this->get_logger(), "\033[1;31mEmergency stop service called\033[0m");
              }
            }


            if (custom_msg.right_controller.gripper == 1.0f && right_gripper == false) {
              auto message = sensor_msgs::msg::JointState();
              message.header.stamp = this->now();
              message.header.frame_id = "base_link";
              
              float gripper_value = 0.05f;
              message.name = {"gripper"};
              message.position = {gripper_value};
              message.velocity = {0.0};  // 使用空数组会导致错误，这里设为0.0
              message.effort = {1.0};
              r_gripper_joint_publisher_->publish(message);
              right_gripper = true;

              RCLCPP_INFO(this->get_logger(), "Right gripper: [%f]", gripper_value);
            }
            else if (custom_msg.right_controller.gripper != 1.0f && right_gripper == true){
              auto message = sensor_msgs::msg::JointState();
              message.header.stamp = this->now();
              message.header.frame_id = "base_link";
              
              float gripper_value = 0.0f;
              message.name = {"gripper"};
              message.position = {gripper_value};
              message.velocity = {0.0};  // 使用空数组会导致错误，这里设为0.0
              message.effort = {1.0};
              r_gripper_joint_publisher_->publish(message);
              right_gripper = false;

              RCLCPP_INFO(this->get_logger(), "Right gripper: [%f]", gripper_value);
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
  // 修改：将 pose_publisher_ 改为 l_joint_publisher_
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr l_joint_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r_joint_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr l_gripper_joint_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr r_gripper_joint_publisher_;

  // 最新左臂真机姿态和相关变量
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr l_real_pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr r_real_pose_subscriber_;
  bool l_real_has_new_pose_ = false;
  bool r_real_has_new_pose_ = false;

  // // 紧急停止
  // rclcpp::Client<std_srvs::srv::Empty>::SharedPtr l_emergency_stop_client_;
  // rclcpp::Client<std_srvs::srv::Empty>::SharedPtr r_emergency_stop_client_;


  bool left_gripper = false;
  bool right_gripper = false;

  bool l_ctl_init = false;
  bool r_ctl_init = false;
  geometry_msgs::msg::PoseStamped l_ctl_init_pose;
  geometry_msgs::msg::PoseStamped l_real_pose;
  geometry_msgs::msg::PoseStamped r_ctl_init_pose;
  geometry_msgs::msg::PoseStamped r_real_pose;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto xrNode = std::make_shared<XRNode>();
  g_callback = [&xrNode] (void* context, PXREAClientCallbackType type,int status,void* userData) {
    xrNode->OnPXREAClientCallback(context, type, status, userData);
  };
  PXREAInit(NULL, callbackForwarder, PXREAFullMask);

  rclcpp::spin(xrNode);

  PXREADeinit();

  rclcpp::shutdown();
  return 0;
}
