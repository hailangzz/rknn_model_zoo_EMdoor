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
  carpet_detect_msgs
)

###################################
## catkin package
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rknn_carpet_detect
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge carpet_detect_msgs
)

###################################
## include dirs
###################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

###################################
## ROS 可执行节点
###################################
add_executable(carpet_detect_listener
  src/main_subscriber_node.cpp
  src/subscriber_operation.cpp
)

add_dependencies(carpet_detect_listener
  ${catkin_EXPORTED_TARGETS}
)
# ============================================================
# Copy librknn_carpet_detect.so to devel/lib/carpet_detect_subscriber
# ============================================================
add_custom_command(TARGET carpet_detect_listener POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory
          ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/lib/librknn_carpet_detect.so
          ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/
  COMMENT "Copy librknn_carpet_detect.so to devel/lib/${PROJECT_NAME}"
)

set_target_properties(carpet_detect_listener PROPERTIES
  BUILD_RPATH "$ORIGIN/../lib"
)

target_link_libraries(carpet_detect_listener
  rknn_carpet_detect
  ${catkin_LIBRARIES}
)
