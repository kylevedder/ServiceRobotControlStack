#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
pushd catkin_ws
catkin_make
popd

pushd rosbuild_ws
export BASE_ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH

pushd robot_status_gui/robot_status_gui
export ROS_PACKAGE_PATH=`pwd`:$BASE_ROS_PACKAGE_PATH
make -j `nproc`
popd

pushd simulator/ut_multirobot_sim
export ROS_PACKAGE_PATH=`pwd`:$BASE_ROS_PACKAGE_PATH
make -j `nproc`
popd

popd
export ROS_PACKAGE_PATH=$BASE_ROS_PACKAGE_PATH

