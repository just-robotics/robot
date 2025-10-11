#!/bin/bash
set -e

cd /microros_ws

source /opt/ros/$ROS_DISTRO/setup.bash

if [ -f "/microros_ws/install/local_setup.bash" ]; then
    source /microros_ws/install/local_setup.bash
    st-flash reset
    bash /microros_ws/run_microros.sh
fi

exec bash
