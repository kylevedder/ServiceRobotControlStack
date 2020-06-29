#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
pushd catkin_ws
catkin_make clean
rm -rf build/ devel/
popd

pushd rosbuild_ws
pushd simulator/ut_multirobot_sim
rm -rf build/ bin/ lib/
popd
popd

