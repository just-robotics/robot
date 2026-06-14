#!/bin/bash
set -e

cd /workspace

source /opt/ros/$ROS_DISTRO/setup.bash

if [ -f "/workspace/install/setup.bash" ]; then
    echo "YEP"
    source /workspace/install/setup.bash
    ros2 launch drive_controller drive_controller.launch.py
else
    echo "NOP"
fi

exec bash
