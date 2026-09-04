#!/bin/bash

# 1. 自动定位到脚本所在目录（防止路径异常）
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

# 2. 加载 ROS 环境
source /opt/ros/noetic/setup.bash

# 3. 循环等待 ROS Master (roscore) 就绪，防止开机时 ROS 还没准备好导致报错
until rostopic list > /dev/null 2>&1; do
  echo "Waiting for ROS Master (roscore)..."
  sleep 1
done

# 4. 设置 ROS 参数
rosparam set /task_manager/carpet_inspection_enable true

# 5. 执行核心运行脚本
exec ./install/lib/carpet_detect_subscriber/run.sh