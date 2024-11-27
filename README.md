
---

## **Kuavo Manipulation 操作说明**

### **1. 启动 NUC 和机器人控制系统**
1. **打开急停按钮下方的 NUC 开机按钮。**
![alt text](<2024-11-26 19-48-09 的屏幕截图.png>)
***roscore -> /home/lab/cam.sh -> /home/lab/hd.sh -> ssh lab@192.168.3.9 -> ssh lemon@192.168.3.27 -> /home/lab/rec.sh***

2. **连接到 NUC：**
   使用 SSH 连接到 NUC：
   ```bash
   ssh lab@192.168.3.9
   # 密码: 输入三个空格
   ```

3. **启动机器人控制：**
   ```bash
   '''
   PC_Terminal_1: roscore
   PC_Terminal_2: roslaunch realsense2_camera rs_camera.launch color_fps:=30 color_height:=480 color_width:=640
   '''

   '''
   PC_Terminal_3: ssh lemon@192.168.3.27 
   '''
   python /home/lemon/robot_ros_application/catkin_ws/src/DynamixelSDK/python/tests/protocol1_0/position_publish_2_for_huawei.py

   '''
   NUC_Terminal_1: 开启上肢状态控制节点
   '''
   sudo su 
   cd /home/lab/syp/kuavo_ws/
   source devel/setup.bash
   rosrun dynamic_biped highlyDynamicRobot_node --real --cali
   # 根据提示按两次“o”, 等待“initial imu Transform succ”后按“v”, 出现“开启ROS进行手臂规划”, 至此打开手臂所有状态发布节点
   
   '''
   NUC_Terminal_2: 调整头部至合适位置，打开正解程序
   '''
   cd /home/lab/syp/kuavo_ws/
   source devel/setup.bash
   conda deactivate
   python3 /home/lab/syp/kuavo_ws/src/motion_capture_ik/scripts/ik_ros_convert.py

   cd /home/lab/syp/kuavo_ws/
   source devel/setup.bash
   conda deactivate
   rostopic pub /robot_head_motion_data dynamic_biped/robotHeadMotionData "{joint_data: [-25, -25.0]}"
   <!-- rostopic pub /robot_head_motion_data dynamic_biped/robotHeadMotionData "{joint_data: [-8, -25.0]}" -->
   
   

   '''
   PC_Terminal_3: ssh lemon@192.168.3.27 遥操作控制
   '''
   python /home/lemon/robot_ros_application/catkin_ws/src/DynamixelSDK/python/tests/protocol1_0/position_publish_2_for_huawei.py
   
   '''
   PC_Terminal_4: 
   source /home/lab/hanxiao/kuavo_ws/devel/setup.zsh
   conda activate robodiff
   python /home/lab/hanxiao/kuavo_ws/src/scripts/hand_srv_to_topic.py
   
   '''

   PC_Terminal_5:
   ```bash
   cd /home/lab/hanxiao/dataset/kuavo/task_test
   rosbag record -o Kuavo_task_test \
             /camera/color/image_raw/compressed \
             /camera/depth/image_rect_raw/compressed \
             /drake_ik/cmd_arm_hand_pose \
             /drake_ik/real_arm_hand_pose \
             /kuavo_arm_traj \
             /robot_arm_q_v_tau \
             /robot_hand_eff \
             /robot_hand_position \
             --bz2 \
             --duration=20 \
             --quiet
   ```

   ```

   - **平行开启新窗口**：按 `<Ctrl B> + "%"`
   - **切换窗口**：按 `<Ctrl B> + "O"`(详见tmux操作手册)
   - **切换到 root 用户**：
     ```bash
     sudo su
     ```
   - **设置 ROS 环境变量**：
     ```bash
     source devel/setup.bash
     ```

4. **注意：** 在此之前，确保 **急停按钮已旋开**。

5. **启动机器人节点：**
   ```bash
   rosrun dynamic_biped highlyDynamicRobot_node2 --real --cali
   ```

6. **校准和启动过程：**
   - 等待系统输出绿色字样提示后，按 **"o"** 键。
   - 再次等待绿色字样提示后，再按 **"o"** 键。
   - 稍等片刻，按 **"v"** 键启动手臂规划。



### **2. 查看 ROS 话题**
1. **列出当前的 ROS 话题：**
   ```bash
   rostopic list
   ```
   输出包括以下几个话题：
   ```
   /control_robot_hand_position  # 灵巧手轨迹发布
   /kuavo_arm_target_poses
   /kuavo_arm_traj              # 手臂轨迹发布
   /leju_robot_phase
   /robot_arm_q_v_tau           # 手臂的状态
   /robot_end_effector_state
   /robot_hand_position         # 灵巧手的状态
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
2. **打开eef topic**
   ```bash
   python3 /home/lab/syp/kuavo_ws/src/motion_capture_ik/scripts/ik_ros_convert.py
   ```
2. **查看某个话题的内容：**
   ```bash
   rostopic echo /robot_arm_q_v_tau
   ```

   **注意：** 如果提示 `ERROR: Cannot load message class for [dynamic_biped/robotArmQVVD]. Are your messages built?`，需要执行以下命令：
   ```bash
   source ~/syp/kuavo_ws/devel/setup.bash
   ```

### **3. 启动 Lemon 的遥操作**
1. **连接到 Lemon：**
   ```bash
   ssh lemon@192.168.1.115
   # 密码: softdev
   ```

2. **启动遥操作：**
   确保遥操作臂已经放置在合适的初始位置，然后运行：
   ```bash
   python /home/lemon/robot_ros_application/catkin_ws/src/DynamixelSDK/python/tests/protocol1_0/position_publish_2_for_huawei.py
   ```

3. **调试：**
   - 如果遥操作臂不动，可以检查并确保控制线连接牢固。

---

**备注：**
- 在进行控制之前，确保所有硬件连接正常，特别是急停按钮状态。
- 保持机器人控制系统的实时监控，以确保每一步都能顺利执行。

## 报错集中
```
[ERROR] [1731511285.818093894]: Client [/hand_srv_to_topic_2262570_1731511285686] wants topic /robot_arm_q_v_tau to have datatype/md5sum [dynamic_biped/robotArmInfo/3871141b674f003bc326e4d8da08f4ad], but our version has [dynamic_biped/robotArmQVVD/a7be9f5331e9207427b0c5c8ace7b977]. Dropping connection.
```
## rosbag record
cd /home/lab/hanxiao/dataset/kuavo/task_test
rosbag record -o Kuavo_task_test \
             /camera/color/image_raw/compressed \
             /camera/depth/image_rect_raw/compressed \
             /drake_ik/cmd_arm_hand_pose \
             /drake_ik/real_arm_hand_pose \
             /kuavo_arm_traj \
             /robot_arm_q_v_tau \
             /robot_hand_eff \
             /robot_hand_position \
             --bz2 \
             --duration=20 \
             --quiet