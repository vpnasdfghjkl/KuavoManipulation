#!/bin/zsh

set -e

# 检查并删除已存在的会话
tmux has-session -t record_sessions 2>/dev/null && tmux kill-session -t record_sessions

# 创建新的会话
tmux new-session -d -s record_sessions

# 检查目标目录并发送命令
if [ -d "/home/lab/hanxiao/dataset/kuavo/task_knife" ]; then
  tmux send-keys -t record_sessions "cd /home/lab/hanxiao/dataset/kuavo/task_toy; python record.py" C-m
else
  echo "目录不存在：/home/lab/hanxiao/dataset/kuavo/task_toy"
  exit 1
fi

# 附加到会话
if ! tmux attach -t record_sessions; then
  echo "无法附加到会话 record_sessions"
  exit 1
fi
