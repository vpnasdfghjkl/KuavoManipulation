#!/bin/zsh

set -e

# 自动登录到 NUC 并执行命令，最后停留在从机上
sshpass -p 'softdev' ssh -tt lemon@192.168.3.27 


