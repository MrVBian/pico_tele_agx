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
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "arm_interfaces/msg/master_controller_command.hpp"
#include "arm_interfaces/action/arm_task.hpp"
#include "arm_interfaces/srv/enable_arm_mode.hpp"



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
    xr_pose_publisher_ = this->create_publisher<arm_interfaces::msg::MasterControllerCommand>("/tele_vr_cmd", 10);
    enable_arm_client_ = this->create_client<arm_interfaces::srv::EnableArmMode>("/arm_enable_servo");
    arm_enable_compliance_client_ = this->create_client<arm_interfaces::srv::EnableArmMode>("/arm_enable_compliance");
    action_client_ = rclcpp_action::create_client<arm_interfaces::action::ArmTask>(this, "/action_manager");

    // 初始化按钮状态
    left_primary_pressed_ = left_secondary_pressed_ = false;
    right_primary_pressed_ = right_secondary_pressed_ = false;
    dual_primary_pressed_ = dual_secondary_pressed_ = false;
    left_primary_press_sent_ = left_secondary_press_sent_ = false;
    right_primary_press_sent_ = right_secondary_press_sent_ = false;
    dual_primary_press_sent_ = dual_secondary_press_sent_ = false;
  }

  ~XRNode() {
  }

  // 使能机械臂服务
  bool EnableArmServo(bool enable) {
    auto request = std::make_shared<arm_interfaces::srv::EnableArmMode::Request>();
    request->enable = enable;
    
    // 等待服务可用
    while (!enable_arm_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
      return false;
    }
    
    // 异步调用服务
    auto result = enable_arm_client_->async_send_request(request);
    
    // 可选：等待结果（如果需要同步调用）
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", 
                  result.get()->message.c_str());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
      return false;
    }
  }
  // 开启力控服务
  bool EnableArmCompliance(bool enable) {
    auto request = std::make_shared<arm_interfaces::srv::EnableArmMode::Request>();
    request->enable = enable;
    
    // 等待服务可用
    while (!arm_enable_compliance_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
      return false;
    }
    
    // 异步调用服务
    auto result = arm_enable_compliance_client_->async_send_request(request);
    
    // 可选：等待结果（如果需要同步调用）
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", 
                  result.get()->message.c_str());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
      return false;
    }
  }

  // 发送双臂伸臂任务 (task_id: 1)
  void SendExtendArmGoal()
  {
    send_arm_task(1);
  }
  void SendLResetArmGoal()
  {
    send_arm_task(100);
  }
  void SendRResetArmGoal()
  {
    send_arm_task(200);
  }
  // 发送双臂收臂
  void SendRetractArmGoal()
  {
    send_arm_task(10);
  }

  void send_arm_task(int task_id)
  {
    // 等待 Action Server 启动（最多等 5 秒）
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server /action_manager not available!");
      return;
    }

    // 创建 Goal
    auto goal = arm_interfaces::action::ArmTask::Goal();
    goal.task_id = task_id;

    RCLCPP_INFO(this->get_logger(), "Sending arm_interfaces::action::ArmTask goal: task_id=%d", task_id);

    // 设置回调函数（可选，去掉也不会报错）
    auto options = typename rclcpp_action::Client<arm_interfaces::action::ArmTask>::SendGoalOptions();
    
    // 结果回调 - 使用最简单的 lambda
    options.result_callback = [this](const rclcpp_action::ClientGoalHandle<arm_interfaces::action::ArmTask>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "ArmTask SUCCEEDED: success=%d, task_id=%d, result='%s'",
                   result.result->success,
                   result.result->task_id,
                   result.result->result.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "arm_interfaces::action::ArmTask FAILED or CANCELED");
      }
    };

    // 发送目标
    action_client_->async_send_goal(goal, options);
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

            tf2::Matrix3x3 rot_mat;
            rot_mat.setValue(0,  0, -1,
                            1, 0,  0,
                            0,  -1,  0);

            tf2::Quaternion q_l_ctl(custom_msg.left_controller.pose[3], custom_msg.left_controller.pose[4], custom_msg.left_controller.pose[5], custom_msg.left_controller.pose[6]);
            tf2::Matrix3x3 R_l_ctl(q_l_ctl);
            tf2::Matrix3x3 R_l_real = rot_mat * R_l_ctl * rot_mat.transpose();
            tf2::Quaternion q_real;
            R_l_real.getRotation(q_real);
            q_real.normalize();

            masterArmCmd.left_command.position.x = -custom_msg.left_controller.pose[2];
            masterArmCmd.left_command.position.y = -custom_msg.left_controller.pose[0];
            masterArmCmd.left_command.position.z = custom_msg.left_controller.pose[1];
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

            tf2::Quaternion q_r_ctl(custom_msg.right_controller.pose[3], custom_msg.right_controller.pose[4], custom_msg.right_controller.pose[5], custom_msg.right_controller.pose[6]);
            tf2::Matrix3x3 R_r_ctl(q_r_ctl);
            tf2::Matrix3x3 R_r_real = rot_mat * R_r_ctl * rot_mat.transpose();
            R_r_real.getRotation(q_real);
            q_real.normalize();

            masterArmCmd.right_command.position.x = -custom_msg.right_controller.pose[2];
            masterArmCmd.right_command.position.y = -custom_msg.right_controller.pose[0];
            masterArmCmd.right_command.position.z = custom_msg.right_controller.pose[1];
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


            // // Y & B 切模式
            // if (custom_msg.left_controller.secondary_button == true && custom_msg.right_controller.secondary_button == true){
            //   masterArmCmd.left_button[3] = true;
            //   masterArmCmd.right_button[3] = true;
            // }
            // // X | A
            // if (custom_msg.left_controller.primary_button || custom_msg.right_controller.primary_button){
            //   masterArmCmd.left_button[4] = true;
            //   masterArmCmd.right_button[4] = true;
            // }


            // ==== 长按检测逻辑 ====
            auto current_time = std::chrono::steady_clock::now();

            // 长按 X & A 双臂伸臂
            if (custom_msg.left_controller.primary_button && custom_msg.right_controller.primary_button) {
              if (!dual_primary_pressed_) {
                dual_primary_pressed_ = true;
                dual_primary_press_start_ = current_time;
                dual_primary_press_sent_ = false;  // 重置发送标志
                // RCLCPP_INFO(this->get_logger(), "Dual primary button pressed, starting timer...");
              } else {
                // 按钮持续按住，检查是否超过pressed_time秒
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - dual_primary_press_start_).count();

                if (duration >= pressed_time && !dual_primary_press_sent_) {
                  RCLCPP_INFO(this->get_logger(), 
                      "Dual primary button held for %ld seconds, sending goal", 
                      duration);
                  dual_primary_press_sent_ = true;  // 标记已发送，防止重复发送
                  SendExtendArmGoal();
                }
              }
            } else {
              // 按钮未被按下，重置状态
              if (dual_primary_pressed_) {
                // RCLCPP_INFO(this->get_logger(), "Dual secondary button released");
              }
              dual_primary_pressed_ = false;
              dual_primary_press_sent_ = false;
            }

            // 左复位
            if (custom_msg.left_controller.primary_button && !custom_msg.right_controller.primary_button) {
              if (!left_primary_pressed_) {
                left_primary_pressed_ = true;
                left_primary_press_start_ = current_time;
                left_primary_press_sent_ = false;  // 重置发送标志
                // RCLCPP_INFO(this->get_logger(), "Left primary button pressed, starting timer...");
              } else {
                // 按钮持续按住，检查是否超过pressed_time秒
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - left_primary_press_start_).count();
                
                if (duration >= pressed_time && !left_primary_press_sent_) {
                  // 长按超过pressed_time秒且未发送过，执行伸臂任务
                  RCLCPP_INFO(this->get_logger(), 
                      "Left primary button held for %ld seconds, sending goal", 
                      duration);
                  left_primary_press_sent_ = true;  // 标记已发送，防止重复发送
                  SendLResetArmGoal();
                }
              }
            } else {
              // 按钮未被按下，重置状态
              if (left_primary_pressed_) {
                // RCLCPP_INFO(this->get_logger(), "Left primary button released");
              }
              left_primary_pressed_ = false;
              left_primary_press_sent_ = false;
            }

            // 右复位
            if (custom_msg.right_controller.primary_button && !custom_msg.left_controller.primary_button) {
              if (!right_primary_pressed_) {
                right_primary_pressed_ = true;
                right_primary_press_start_ = current_time;
                right_primary_press_sent_ = false;  // 重置发送标志
                // RCLCPP_INFO(this->get_logger(), "Right primary button pressed, starting timer...");
              } else {
                // 按钮持续按住，检查是否超过pressed_time秒
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - right_primary_press_start_).count();
                
                if (duration >= pressed_time && !right_primary_press_sent_) {
                  // 长按超过pressed_time秒且未发送过，执行收臂任务
                  RCLCPP_INFO(this->get_logger(), 
                      "Right primary button held for %ld seconds, sending goal", 
                      duration);
                  right_primary_press_sent_ = true;  // 标记已发送，防止重复发送
                  SendRResetArmGoal();
                }
              }
            } else {
              // 按钮未被按下，重置状态
              if (right_primary_pressed_) {
                // RCLCPP_INFO(this->get_logger(), "Right primary button released");
              }
              right_primary_pressed_ = false;
              right_primary_press_sent_ = false;
            }


            // 长按 Y & B 使能
            if (custom_msg.left_controller.secondary_button && custom_msg.right_controller.secondary_button) {
              if (!dual_secondary_pressed_) {
                dual_secondary_pressed_ = true;
                dual_secondary_press_start_ = current_time;
                dual_secondary_press_sent_ = false;  // 重置发送标志
                // RCLCPP_INFO(this->get_logger(), "Dual secondary button pressed, starting timer...");
              } else {
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - dual_secondary_press_start_).count();

                if (duration >= pressed_time && !dual_secondary_press_sent_) {
                  RCLCPP_INFO(this->get_logger(), 
                      "Dual secondary button held for %ld seconds, sending goal", 
                      duration);
                  dual_secondary_press_sent_ = true;  // 标记已发送，防止重复发送
                  EnableArmServo(true);
                }
              }
            } else {
              // 按钮未被按下，重置状态
              if (dual_secondary_pressed_) {
                // RCLCPP_INFO(this->get_logger(), "Dual secondary button released");
              }
              dual_secondary_pressed_ = false;
              dual_secondary_press_sent_ = false;
            }

            // Y 双臂收臂
            if (custom_msg.left_controller.secondary_button && !custom_msg.right_controller.secondary_button) {
              if (!left_secondary_pressed_) {
                left_secondary_pressed_ = true;
                left_secondary_press_start_ = current_time;
                left_secondary_press_sent_ = false;  // 重置发送标志
                // RCLCPP_INFO(this->get_logger(), "Left secondary button pressed, starting timer...");
              } else {
                // 按钮持续按住，检查是否超过pressed_time秒
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - left_secondary_press_start_).count();
                
                if (duration >= pressed_time && !left_secondary_press_sent_) {
                  // 长按超过pressed_time秒且未发送过，执行伸臂任务
                  RCLCPP_INFO(this->get_logger(), 
                      "Left secondary button held for %ld seconds, sending goal", 
                      duration);
                  left_secondary_press_sent_ = true;  // 标记已发送，防止重复发送
                  SendRetractArmGoal();
                }
              }
            } else {
              // 按钮未被按下，重置状态
              if (left_secondary_pressed_) {
                // RCLCPP_INFO(this->get_logger(), "Left secondary button released");
              }
              left_secondary_pressed_ = false;
              left_secondary_press_sent_ = false;
            }

            // B 开启力控
            if (custom_msg.right_controller.secondary_button && !custom_msg.left_controller.secondary_button) {
              if (!right_secondary_pressed_) {
                right_secondary_pressed_ = true;
                right_secondary_press_start_ = current_time;
                right_secondary_press_sent_ = false;  // 重置发送标志
                // RCLCPP_INFO(this->get_logger(), "Right secondary button pressed, starting timer...");
              } else {
                // 按钮持续按住，检查是否超过pressed_time秒
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - right_secondary_press_start_).count();
                
                if (duration >= pressed_time && !right_secondary_press_sent_) {
                  // 长按超过pressed_time秒且未发送过，执行收臂任务
                  RCLCPP_INFO(this->get_logger(), 
                      "Right secondary button held for %ld seconds, sending goal", 
                      duration);
                  right_secondary_press_sent_ = true;  // 标记已发送，防止重复发送
                  EnableArmCompliance(true);
                }
              }
            } else {
              // 按钮未被按下，重置状态
              if (right_secondary_pressed_) {
                // RCLCPP_INFO(this->get_logger(), "Right secondary button released");
              }
              right_secondary_pressed_ = false;
              right_secondary_press_sent_ = false;
            }

            // // 输出日志：记录最终发送给双臂的命令数据
            // RCLCPP_INFO(this->get_logger(),
            //     "Master Arm Commands Updated:\n"
            //     "Left  -> Position: [%.4f, %.4f, %.4f], Orientation: [%.4f, %.4f, %.4f, %.4f]\n"
            //     "Right -> Position: [%.4f, %.4f, %.4f], Orientation: [%.4f, %.4f, %.4f, %.4f]",
            //     masterArmCmd.left_command.position.x,
            //     masterArmCmd.left_command.position.y,
            //     masterArmCmd.left_command.position.z,
            //     masterArmCmd.left_command.orientation.x,
            //     masterArmCmd.left_command.orientation.y,
            //     masterArmCmd.left_command.orientation.z,
            //     masterArmCmd.left_command.orientation.w,
            //     masterArmCmd.right_command.position.x,
            //     masterArmCmd.right_command.position.y,
            //     masterArmCmd.right_command.position.z,
            //     masterArmCmd.right_command.orientation.x,
            //     masterArmCmd.right_command.orientation.y,
            //     masterArmCmd.right_command.orientation.z,
            //     masterArmCmd.right_command.orientation.w
            // );

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
  rclcpp::Client<arm_interfaces::srv::EnableArmMode>::SharedPtr enable_arm_client_;
  rclcpp::Client<arm_interfaces::srv::EnableArmMode>::SharedPtr arm_enable_compliance_client_;
  rclcpp_action::Client<arm_interfaces::action::ArmTask>::SharedPtr action_client_;

  bool real_has_new_pose_ = false;

  bool l_ctl_init = false;
  bool r_ctl_init = false;
  geometry_msgs::msg::PoseStamped l_ctl_init_pose;
  geometry_msgs::msg::PoseStamped l_real_pose;
  geometry_msgs::msg::PoseStamped r_ctl_init_pose;
  geometry_msgs::msg::PoseStamped r_real_pose;

  // ==== 长按检测相关变量 ====
  int pressed_time = 1;
  bool left_primary_pressed_;
  bool right_primary_pressed_;
  bool dual_primary_pressed_;
  bool left_primary_press_sent_;
  bool right_primary_press_sent_;
  bool dual_primary_press_sent_;
  bool left_secondary_pressed_;
  bool right_secondary_pressed_;
  bool dual_secondary_pressed_;
  bool left_secondary_press_sent_;
  bool right_secondary_press_sent_;
  bool dual_secondary_press_sent_;
  std::chrono::steady_clock::time_point left_primary_press_start_;
  std::chrono::steady_clock::time_point right_primary_press_start_;
  std::chrono::steady_clock::time_point dual_primary_press_start_;
  std::chrono::steady_clock::time_point left_secondary_press_start_;
  std::chrono::steady_clock::time_point right_secondary_press_start_;
  std::chrono::steady_clock::time_point dual_secondary_press_start_;
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

