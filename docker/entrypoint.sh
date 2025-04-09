#! /bin/bash


source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WS}/install/setup.bash
source ${WS_drivers_stm}/install/local_setup.bash

#ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 &
ros2 launch drive_controller drive_controller.launch.py &

exec bash
