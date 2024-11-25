#!/bin/zsh

set -e

# 删除已存在的会话（忽略错误）
tmux kill-session -t hand_sessions 2>/dev/null || true

# 创建新的 tmux 会话
tmux new-session -d -s hand_sessions

# 在会话中启动 roslaunch 脚本
tmux send-keys -t hand_sessions \
  "source /home/lab/hanxiao/kuavo_ws/devel/setup.zsh; \
   conda activate robodiff; \
   python /home/lab/hanxiao/kuavo_ws/src/scripts/hand_srv_to_topic.py" C-m

# 附加到会话
tmux attach -t hand_sessions
