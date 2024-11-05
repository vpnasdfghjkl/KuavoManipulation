# KuavoManipulation
## Kuavo
1. 打开急停下面的NUC开机按钮
2. 连接NUC,开启机器人手臂控制
```bash
ssh lab@192.168.1.116
# password: 三个空格

cd ~/syp/kuavo_ws/

tmux new -s kuavo

roscore
<ctrl B> + "%" 平行开启新窗口
sudo su 
source devel/setup.bash
在此之前旋开急停
rosrun dynamic_biped highlyDynamicRobot_node2 --real --cali
等待绿色字样提示后：按"o"键
等待绿色字样提示后：再按"o"键
稍等片刻，按"v"键开启手臂规划
```
<ctrl B> + "O" 切换窗口
```
3. 查看rostopic
```bash
lab@NUC11TNKi7:~/syp/kuavo_ws$ rostopic list
/control_robot_hand_position # 灵巧手轨迹发布
/kuavo_arm_target_poses
/kuavo_arm_traj # 手臂轨迹的发布
/leju_robot_phase
/robot_arm_q_v_tau # 手臂的状态
/robot_end_effector_state
/robot_hand_position    # 灵巧手的状态
/robot_head_motion_data
/robot_head_motor_position
/robot_imu_gyro_acc
/robot_plan_arm_state
/robot_q_v_tau
/robot_torso_state
/rosout
/rosout_agg
/walkCommand
```
4. 查看rostopic内容
```bash
# 提示"ERROR: Cannot load message class for [dynamic_biped/robotArmQVVD]. Are your messages built?" 时，需要source
source ~/syp/kuavo_ws/devel/setup.bash
rostopic echo /robot_arm_q_v_tau
```

## Lemon
```bash
ssh lemon@192.168.1.115
# password: softdev

# 打开遥操作(之前注意把遥操作臂放到合适的初始位置)
python /home/lemon/robot_ros_application/catkin_ws/src/DynamixelSDK/python/tests/protocol1_0/position_publish_2_for_huawei.py
# 之后就可以操作了如果遥操作臂不动，可以把线插紧试试

```