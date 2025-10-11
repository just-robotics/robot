# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

rm -rf /microros_ws/src/uros/*
rm -rf /microros_ws/src/ros2.repos

# Create a workspace and download the micro-ROS tools
# git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

#ros2 run micro_ros_setup create_firmware_ws.sh host
#ros2 run micro_ros_setup build_firmware.sh
#source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh

ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
