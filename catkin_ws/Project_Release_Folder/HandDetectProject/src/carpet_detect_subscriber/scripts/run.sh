#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
INSTALL_PREFIX=$(cd "${SCRIPT_DIR}/../.." && pwd)
PKG_SHARE=${INSTALL_PREFIX}/share/carpet_detect_subscriber

echo "roscore 已在运行"
echo "切换到包资源目录: ${PKG_SHARE}"

# 切换工作目录
cd ${PKG_SHARE}

# 导入动态库文件
export LD_LIBRARY_PATH=${PKG_SHARE}/lib:$LD_LIBRARY_PATH

# 设置ROS环境
source ${INSTALL_PREFIX}/setup.bash

echo "启动 main_carpet_detect_subscriber_node..."
exec ${SCRIPT_DIR}/main_carpet_detect_subscriber_node
