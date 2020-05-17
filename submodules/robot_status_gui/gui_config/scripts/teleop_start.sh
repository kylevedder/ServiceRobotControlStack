#!/usr/bin/env bash
source devel/setup.bash
screen -S teleop_node -dm sh -c 'rosrun control_stack teleop_node; exec bash' &