#!/bin/bash
set -e

cd /workspace

source /opt/ros/$ROS_DISTRO/setup.bash

roscore &

if [ -f "/workspace/devel/setup.bash" ]; then
    source /workspace/devel/setup.bash
fi

exec bash
