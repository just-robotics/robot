#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/ros/noetic/setup.bash

ros2 run ros1_bridge dynamic_bridge

exec bash
