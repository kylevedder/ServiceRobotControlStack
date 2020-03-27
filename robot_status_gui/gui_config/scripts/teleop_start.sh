#!/usr/bin/env bash
source devel/setup.bash
screen -S teleop_node -dm rosrun control_stack teleop_node &