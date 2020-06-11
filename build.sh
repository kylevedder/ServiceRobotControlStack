#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
failed=0
pushd catkin_ws
catkin_make
if [ "$?" -ne "0" ]; then failed=1; fi
devel/lib/control_stack/control_stack_unit_tests
if [ "$?" -ne "0" ]; then failed=1; fi
popd

if [ "$failed" -ne "0" ]; then exit 1; fi

pushd rosbuild_ws
export BASE_ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH

pushd simulator/ut_multirobot_sim
export ROS_PACKAGE_PATH=`pwd`:$BASE_ROS_PACKAGE_PATH
make -j `nproc`
if [ "$?" -ne "0" ]; then failed=1; fi
popd

popd
export ROS_PACKAGE_PATH=$BASE_ROS_PACKAGE_PATH
if [ "$failed" -ne "0" ]; then exit 1; fi

