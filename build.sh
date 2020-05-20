#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
pushd catkin_ws
catkin_make
popd
