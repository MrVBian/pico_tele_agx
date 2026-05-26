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

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "arm_interfaces/msg/master_controller_command.hpp"
#include "arm_interfaces/msg/arm_status.hpp"


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
    
    // real
    xr_pose_publisher_ = this->create_publisher<arm_interfaces::msg::MasterControllerCommand>("/tele_vr_cmd", 10);

    real_pose_subscriber_ = this->create_subscription<arm_interfaces::msg::ArmStatus>("/arm_status", 10, std::bind(&XRNode::PoseCallback, this, std::placeholders::_1));
    void PoseCallback(const arm_interfaces::msg::ArmStatus::SharedPtr msg);
  }

  ~XRNode() {
  }

  void PoseCallback(const arm_interfaces::msg::ArmStatus::SharedPtr msg)
  {
    (void)msg;
    if (real_has_new_pose_ == false){
      real_has_new_pose_ = true;

      // // 打印接收到的信息
      // RCLCPP_INFO(this->get_logger(), "\033[1;33mTCP位姿 topic: %s\n- 位置: [x: %.3f, y: %.3f, z: %.3f]\n姿态: [qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f]\033[0m", 
      //             l_real_pose_subscriber_->get_topic_name(),
      //             l_real_pose.pose.position.x, l_real_pose.pose.position.y, l_real_pose.pose.position.z,
      //             l_real_pose.pose.orientation.x, l_real_pose.pose.orientation.y, 
      //             l_real_pose.pose.orientation.z, l_real_pose.pose.orientation.w
      // );
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

            // XR
            publisher_->publish(custom_msg);


            auto masterArmCmd = arm_interfaces::msg::MasterControllerCommand();
            masterArmCmd.header.stamp = rclcpp::Node::now();



            // 构造控制器在原始坐标系（与控制器坐标轴定义相同）中的变换
            tf2::Transform T_l_ctl_W;
            T_l_ctl_W.setOrigin(tf2::Vector3(custom_msg.left_controller.pose[0], custom_msg.left_controller.pose[1], custom_msg.left_controller.pose[2]));
            tf2::Quaternion q_l_ctl(custom_msg.left_controller.pose[3], custom_msg.left_controller.pose[4], custom_msg.left_controller.pose[5], custom_msg.left_controller.pose[6]);
            T_l_ctl_W.setRotation(q_l_ctl);

            // 定义从原始坐标系到真实世界坐标系的旋转矩阵
            // 根据映射关系：控制器(x右,y上,z后) -> 真实世界(x前,y左,z上)
            // 对应的旋转矩阵为：
            // [0,  0, -1;
            //  -1, 0,  0;
            //  0,  1,  0]
            tf2::Matrix3x3 rot_mat;
            rot_mat.setValue(0,  0, -1,
                            -1, 0,  0,
                            0,  1,  0);
            // 构造从原始坐标系到真实世界坐标系的变换（仅旋转，无平移）
            tf2::Transform T_W_to_real;
            T_W_to_real.setBasis(rot_mat);
            T_W_to_real.setOrigin(tf2::Vector3(0, 0, 0));

            // 计算控制器在真实世界坐标系下的变换
            tf2::Transform T_ctl_real = T_W_to_real * T_l_ctl_W;

            // 提取真实世界坐标系下的位置和四元数
            tf2::Vector3 p_real = T_ctl_real.getOrigin();
            tf2::Quaternion q_real = T_ctl_real.getRotation();

            masterArmCmd.left_command.position.x = p_real.x();
            masterArmCmd.left_command.position.y = p_real.y();
            masterArmCmd.left_command.position.z = p_real.z();
            masterArmCmd.left_command.orientation.x = q_real.x();
            masterArmCmd.left_command.orientation.y = q_real.y();
            masterArmCmd.left_command.orientation.z = q_real.z();
            masterArmCmd.left_command.orientation.w = q_real.w();

            if (custom_msg.left_controller.gripper==1.0f){
              masterArmCmd.left_button[9] = true;
            }
            else{
              masterArmCmd.left_button[9] = false;
            }
            if (custom_msg.left_controller.trigger == 1.0f){
              masterArmCmd.left_button[13] = false;
              masterArmCmd.left_button[14] = true;
            }
            else{
              masterArmCmd.left_button[13] = true;
              masterArmCmd.left_button[14] = false;
            }

            tf2::Transform T_r_ctl_W;
            T_r_ctl_W.setOrigin(tf2::Vector3(custom_msg.right_controller.pose[0], custom_msg.right_controller.pose[1], custom_msg.right_controller.pose[2]));
            tf2::Quaternion q_r_ctl(custom_msg.right_controller.pose[3], custom_msg.right_controller.pose[4], custom_msg.right_controller.pose[5], custom_msg.right_controller.pose[6]);
            T_r_ctl_W.setRotation(q_r_ctl);
            // 计算控制器在真实世界坐标系下的变换
            T_ctl_real = T_W_to_real * T_r_ctl_W;
            // 提取真实世界坐标系下的位置和四元数
            p_real = T_ctl_real.getOrigin();
            q_real = T_ctl_real.getRotation();
            masterArmCmd.right_command.position.x = p_real.x();
            masterArmCmd.right_command.position.y = p_real.y();
            masterArmCmd.right_command.position.z = p_real.z();
            masterArmCmd.right_command.orientation.x = q_real.x();
            masterArmCmd.right_command.orientation.y = q_real.y();
            masterArmCmd.right_command.orientation.z = q_real.z();
            masterArmCmd.right_command.orientation.w = q_real.w();

            if (custom_msg.right_controller.gripper==1.0f){
              masterArmCmd.right_button[9] = true;
            }
            else{
              masterArmCmd.right_button[9] = false;
            }
            if (custom_msg.right_controller.trigger == 1.0f){
              masterArmCmd.right_button[13] = false;
              masterArmCmd.right_button[14] = true;
            }
            else{
              masterArmCmd.right_button[13] = true;
              masterArmCmd.right_button[14] = false;
            }


            if (custom_msg.left_controller.primary_button == true && custom_msg.right_controller.primary_button == true){
              masterArmCmd.left_button[3] = true;
              masterArmCmd.right_button[3] = true;
            }
            if (custom_msg.left_controller.secondary_button || custom_msg.right_controller.secondary_button){
              masterArmCmd.left_button[4] = true;
              masterArmCmd.right_button[4] = true;
            }


            // 输出日志：记录最终发送给双臂的命令数据
            RCLCPP_INFO(this->get_logger(),
                "Master Arm Commands Updated:\n"
                "Left  -> Position: [%.4f, %.4f, %.4f], Orientation: [%.4f, %.4f, %.4f, %.4f]\n"
                "Right -> Position: [%.4f, %.4f, %.4f], Orientation: [%.4f, %.4f, %.4f, %.4f]",
                masterArmCmd.left_command.position.x,
                masterArmCmd.left_command.position.y,
                masterArmCmd.left_command.position.z,
                masterArmCmd.left_command.orientation.x,
                masterArmCmd.left_command.orientation.y,
                masterArmCmd.left_command.orientation.z,
                masterArmCmd.left_command.orientation.w,
                masterArmCmd.right_command.position.x,
                masterArmCmd.right_command.position.y,
                masterArmCmd.right_command.position.z,
                masterArmCmd.right_command.orientation.x,
                masterArmCmd.right_command.orientation.y,
                masterArmCmd.right_command.orientation.z,
                masterArmCmd.right_command.orientation.w
            );

            // std::cout << "抓握:" << (masterArmCmd.left_button[9] ? "true" : "false")
            //   << ", 扳机:" << (masterArmCmd.left_button[14] ? "true" : "false")
            //   << ", X&A:" << (masterArmCmd.left_button[3] ? "true" : "false")
            //   << ", Y/B:" << (masterArmCmd.left_button[4] ? "true" : "false")
            //   << std::endl;

            xr_pose_publisher_->publish(masterArmCmd);

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
  rclcpp::Publisher<arm_interfaces::msg::MasterControllerCommand>::SharedPtr xr_pose_publisher_;
  // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r_joint_publisher_;
  rclcpp::Subscription<arm_interfaces::msg::ArmStatus>::SharedPtr real_pose_subscriber_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr r_gripper_joint_publisher_;

  bool real_has_new_pose_ = false;

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
