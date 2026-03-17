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
  LIBRARIES rknn_carpet_detect
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
## rknn_carpet_detect 动态库（⚠️ 排除 main）
###################################
file(GLOB YOLO_SRCS
  src/*.cpp
)

# ⚠️ 关键：把 main 从库里移除
list(REMOVE_ITEM YOLO_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main_subscriber_node.cpp
)

add_library(rknn_carpet_detect SHARED
  ${YOLO_SRCS}
)

target_link_libraries(rknn_carpet_detect
  ${catkin_LIBRARIES}
)


