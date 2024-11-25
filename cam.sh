#!/bin/zsh

set -e

# 检查是否存在 ros_sessions 会话，如果存在则删除它
tmux kill-session -t ros_sessions 2>/dev/null || true

# 创建一个新的 tmux 会话并运行 roslaunch 命令
tmux new-session -d -s ros_sessions

# 在第一个窗格中启动 roslaunch
tmux send-keys -t ros_sessions "roslaunch realsense2_camera rs_multiple_devices.launch" C-m

# 创建一个水平分屏并在右侧启动 rviz
tmux split-window -h
tmux send-keys -t ros_sessions "rviz" C-m  # 在第一个窗格（即分屏右侧）启动 rviz

# # 创建一个新的垂直分屏并在下方运行 bash 脚本
# #tmux split-window -v
# #tmux send-keys -t ros_sessions "bash /home/lab/hand_srv_to_topic.sh" C-m

# # 附加到 tmux 会话，查看所有窗口和分屏
tmux attach -t ros_sessions


