#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
pushd catkin_ws
catkin_make clean
rm -rf build/ devel/
popd

pushd rosbuild_ws
pushd robot_status_gui/robot_status_gui
rm -rf build/ bin/ lib/
popd

pushd simulator/f1tenth_simulator
rm -rf build/ bin/ lib/
popd

popd

