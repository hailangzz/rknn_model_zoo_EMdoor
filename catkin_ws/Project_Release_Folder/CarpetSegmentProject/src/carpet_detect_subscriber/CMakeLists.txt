cmake_minimum_required(VERSION 3.0.2)
project(carpet_detect_subscriber)

# ------------------------------------------------
# 找 ROS & 依赖包
# ------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  carpet_detect_msgs
)

# OpenCV（cv_bridge 依赖）
find_package(OpenCV REQUIRED)

# ------------------------------------------------
# catkin package 声明
# ------------------------------------------------
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    carpet_detect_msgs
)

# ------------------------------------------------
# 头文件目录
# ------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ------------------------------------------------
# 可执行程序
# ------------------------------------------------
add_executable(main_carpet_detect_subscriber_node
  src/main_subscriber_node.cpp
  src/subscriber_operation.cpp
)

set_target_properties(main_carpet_detect_subscriber_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../share/carpet_detect_subscriber/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# ------------------------------------------------
# 链接库
# ------------------------------------------------
target_link_libraries(main_carpet_detect_subscriber_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}

  # 第三方 RKNN / RGA
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/librknn_carpet_detect.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/librknnrt.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/librga.so
)

# ------------------------------------------------
# 依赖（消息 / catkin）
# ------------------------------------------------
add_dependencies(main_carpet_detect_subscriber_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ------------------------------------------------
# install 规则（非常关键）
# ------------------------------------------------

# 1️⃣ 安装可执行程序
install(TARGETS main_carpet_detect_subscriber_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 2️⃣ 安装脚本（run.sh）
install(PROGRAMS
  scripts/run.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 3️⃣ 安装模型 / 配置 / so（强烈推荐）
install(DIRECTORY
  model
  config
  lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
