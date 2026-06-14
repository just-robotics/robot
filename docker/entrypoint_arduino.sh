#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash

rosrun rosserial_arduino serial_node.py /dev/arduino

exec bash
