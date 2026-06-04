std_msgs/Header header
geometry_msgs/Pose left_command
geometry_msgs/Pose right_command
uint8[10] int_command
bool[32] left_button
bool[32] right_button
bool signal

# 
# left/right_command
# 
# int_command
# 0:左手柄夹爪行程
# 1:右手柄夹爪行程
# 2:脚踏板指令
# 3:
# 
# left_button/right_button
# 
# 抓握键 [9]
# 扳机键按下 [14]
# 扳机键松开 [13]
# X&A [3]
# Y/B [4]
# 
# signal
# 说明该帧指令是否需要发送