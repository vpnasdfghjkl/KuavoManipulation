好的，这里是你提供的操作说明的整理版：

---

## **Kuavo Manipulation 操作说明**

### **1. 启动 NUC 和机器人控制系统**
1. **打开急停按钮下方的 NUC 开机按钮。**
2. **连接到 NUC：**
   使用 SSH 连接到 NUC：
   ```bash
   ssh lab@192.168.1.116
   # 密码: 输入三个空格
   ```

3. **启动机器人控制：**
   ```bash
   cd ~/syp/kuavo_ws/
   tmux new -s kuavo  # 创建一个新的 tmux 会话
   roscore           # 启动 ROS 核心
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