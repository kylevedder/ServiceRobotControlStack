#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
pushd catkin_ws
rm -rf build/ devel/
popd

pushd rosbuild_ws
pushd robot_status_gui/robot_status_gui
rm -rf build/ bin/ lib/
popd
popd

