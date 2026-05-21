Pico仿真

```shell
ros2 run picoxr talker
ros2 run ik_solver_py ik_example

ros2 launch agx_arm_ctrl rviz_zmebot_description.launch.py

ros2 launch agx_arm_description display.launch.py arm_type:=piper

ros2 topic pub /joint_states sensor_msgs/msg/JointState "
header:
  stamp: now
  frame_id: ''
name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
position: [0.3, 0.3, 0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: []
" --once

ros2 topic pub /control/joint_states sensor_msgs/msg/JointState '
name: ["gripper"]
position: [0.00]
velocity: []
effort: [1.0]
' -1

ros2 topic pub /feedback/tcp_pose geometry_msgs/msg/PoseStamped "
header:
  stamp: {sec: $(date +%s), nanosec: 0}
  frame_id: ''
pose:
  position: {x: 0.3, y: 0.0, z: 0.2}
  orientation: {x: 0.0, y: 0.6755837736314217, z: 0.0, w: 0.7372832324188093}
" -1

ros2 topic pub /feedback/tcp_pose geometry_msgs/msg/PoseStamped "
header:
  stamp: {sec: $(date +%s), nanosec: 0}
  frame_id: ''
pose:
  position: {x: 0.3, y: 0.0, z: 0.4}
  orientation: {x: 0.0, y: 0.6755837736314217, z: 0.0, w: 0.7372832324188093}
" -1

ros2 topic pub /xr_pose xr_msgs/msg/Custom "
left_controller:
  axis_x: 0.0
  axis_y: 0.0
  axis_click: true
  gripper: 1.0
  trigger: 0.0
  primary_button: false
  secondary_button: false
  menu_button: false
  pose: [0.3, 0.0, 0.4, 0.199, 0.651, 0.182, 0.710]
  status: 0
"
```

Pico真机

```shell
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=none tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'

ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'

ros2 run picoxr talker

ros2 topic echo /control/move_p geometry_msgs/msg/PoseStamped


ros2 launch agx_arm_ctrl start_dual_agx_arm.launch.py   left_can_port:=can_left   right_can_port:=can_right   arm_type:=piper   left_effector_type:=agx_gripper   right_effector_type:=agx_gripper   tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'

左臂 Interface can0 is connected to USB port 1-3:1.0
右臂 Interface can0 is connected to USB port 1-3:1.0


ros2 topic pub /left/control/move_p geometry_msgs/msg/PoseStamped \
  "$(cat test/piper/test_move_p.yaml)" -1
```

can通信

```bash
sudo apt update && sudo apt install can-utils ethtool

cd /home/zme/project/ros/xr/agx_arm_ws/src/agx_arm_ros/scripts
bash find_all_can_port.sh

bash can_activate.sh can0 1000000

wk
accan
```

- 场景 4：真机 + 控制 + 跟随

  （从 RViz 发控制，并在 RViz 跟随真实反馈）

  - 是否需要真机：是
  - 推荐配置：`follow:=true, control:=true`
  - 示例：

```shell
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can0 arm_type:=piper follow:=true control:=true
```

**MoveIt 一键启动（臂控 + MoveIt + RViz）：**

```shell
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper
```

点到点控制

```shell
ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
  "{name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], position: [0.0, 0.2, -0.2, 0.0, -0.0, 0.0], velocity: [], effort: []}" -1
  
  
  
 header:
  stamp:
    sec: 1777512899
    nanosec: 366549070
  frame_id: Pico
pose:
  position:
    x: 0.09283056855201721
    y: 0.23525045812129974
    z: 0.14995598793029785
  orientation:
    x: 0.18691378878357262
    y: -0.3279682704023676
    z: 0.25291785660713245
    w: 0.8908044718012497
 
  
ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.09283056855201721, y: 0.23525045812129974, z: 0.14995598793029785}
  orientation: {x: 0.18691378878357262, y: -0.3279682704023676, z: 0.25291785660713245, w: 0.8908044718012497}
" -1



ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped "
header:
  stamp: {sec: $(date +%s), nanosec: 0}
  frame_id: ''
pose:
  position: {x: 0.2, y: 0.0, z: 0.3}
  orientation: {x: 0.0, y: 0.6755837736314217, z: 0.0, w: 0.7372832324188093}
" -1



ros2 topic echo /control/move_p
ros2 topic pub /left/control/move_p geometry_msgs/msg/PoseStamped "
header:
  stamp: {sec: $(date +%s), nanosec: 0}
  frame_id: ''
pose:
  position: {x: 0.3, y: 0.0, z: 0.3}
  orientation: {x: 0.0, y: 0.6755837736314217, z: 0.0, w: 0.7372832324188093}
" -1

ros2 topic pub /right/control/move_p geometry_msgs/msg/PoseStamped "
header:
  stamp: {sec: $(date +%s), nanosec: 0}
  frame_id: ''
pose:
  position: {x: 0.3, y: 0.0, z: 0.3}
  orientation: {x: 0.0, y: 0.6755837736314217, z: 0.0, w: 0.7372832324188093}
" -1
```

### 服务调用

1. 使能机械臂

```shell
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
```

1. 失能机械臂

```shell
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
```

1. 回零位

```shell
ros2 service call /move_home std_srvs/srv/Empty
```

1. 急停（保持当前位置）

```shell
ros2 service call /emergency_stop std_srvs/srv/Empty
```

1. 退出示教模式（Piper 系列）

```shell
ros2 service call /exit_teach_mode std_srvs/srv/Empty
```

> **⚠️ 重要安全提示:**
>
> 1. 执行该指令后，机械臂会先执行回零位操作，随后自动重启；此过程中机械臂存在坠落风险，建议在回零位完成后用手轻扶机械臂，防止坠落损坏。
> 2. Piper 系列机器臂若固件版本为 1.8.5 及以上，已支持 模式无缝切换 功能，无需执行上述退出示教模式的服务指令，系统会自动完成模式切换，可规避上述坠落风险。

custom_msg.left_controller是右手系，x右正、y上正、z后正
 ps.pose是右手系，x前正、y左正、z上正
 在赋值时，需要转换至ps.pose右手系的坐标系下，修改代码

```cpp
ps.pose.position.x = -custom_msg.left_controller.pose[2];
ps.pose.position.y = -custom_msg.left_controller.pose[0];
ps.pose.position.z = custom_msg.left_controller.pose[1];
// 四元数转换（controller -> ps 坐标系）
auto qconv = convertControllerQuatToPoseQuat(
custom_msg.left_controller.pose[3],
custom_msg.left_controller.pose[4],
custom_msg.left_controller.pose[5],
custom_msg.left_controller.pose[6]
);
ros2 pkg create ik_solver   --build-type ament_cmake   --dependencies rclcpp geometry_msgs pinocchio

locate libcurl.so.4
sudo rm /usr/local/lib/libcurl.so.4
sudo ln -s /usr/lib/x86_64-linux-gnu/libcurl.so.4.7.0 /usr/local/lib/libcurl.so.4

colcon build --packages-select ik_solver

ros2 run ik_solver ik_solver_node
```
