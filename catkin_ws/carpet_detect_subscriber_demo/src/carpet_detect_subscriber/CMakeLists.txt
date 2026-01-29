cmake_minimum_required(VERSION 3.0.2)
project(carpet_detect_subscriber)

add_compile_options(-std=c++14)

###################################
## find catkin
###################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
)

###################################
## catkin package
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yolov8
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge
)

###################################
## include dirs
###################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###################################
## YOLOv8 动态库（⚠️ 排除 main）
###################################
file(GLOB YOLO_SRCS
  src/*.cpp
)

# ⚠️ 关键：把 main 从库里移除
list(REMOVE_ITEM YOLO_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main_subscriber_node.cpp
)

add_library(yolov8 SHARED
  ${YOLO_SRCS}
)

target_link_libraries(yolov8
  ${catkin_LIBRARIES}
)

###################################
## ROS 可执行节点
###################################
add_executable(carpet_detect_listener
  src/main_subscriber_node.cpp
)

target_link_libraries(carpet_detect_listener
  yolov8
  ${catkin_LIBRARIES}
)
