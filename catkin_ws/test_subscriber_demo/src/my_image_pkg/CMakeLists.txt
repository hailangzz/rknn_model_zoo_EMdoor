cmake_minimum_required(VERSION 3.0.2)
project(my_image_pkg)  # 必须先调用 project() 定义包名

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_image_pkg
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_listener src/image_subscriber_node.cpp)  # image_listener 为可执行程序名称
target_link_libraries(image_listener ${catkin_LIBRARIES})   
