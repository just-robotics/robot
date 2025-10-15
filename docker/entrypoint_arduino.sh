#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash

if [ -f "/workspace/devel/setup.bash" ]; then
    source /workspace/devel/setup.bash
fi

cd /arduino/libraries
rosrun rosserial_arduino make_libraries.py .

cd /workspace

exec bash