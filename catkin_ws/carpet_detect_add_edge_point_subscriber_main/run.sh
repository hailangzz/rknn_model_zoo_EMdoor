cd /home/robot/zhangzhuo/catkin_wc

source /opt/ros/noetic/setup.bash

rosparam get /task_manager/carpet_inspection_enable

rosparam set /task_manager/carpet_inspection_enable true

source devel/setup.bash

rosrun carpet_detect_subscriber carpet_detect_listener
