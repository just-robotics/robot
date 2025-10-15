#!/bin/bash
set -e

cd /workspace

source /opt/ros/$ROS_DISTRO/setup.bash

if [ -f "/workspace/install/local_setup.bash" ]; then
    source /workspace/install/local_setup.bash
    ros2 run ros1_bridge dynamic_bridge
fi

exec bash
