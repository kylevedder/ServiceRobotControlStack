# Service Robot Control Stack

Full planning and control stack for GRASP LML Service Robots @ Penn.

[![Build Status](https://travis-ci.com/kylevedder/ServiceRobotControlStack.svg?branch=master)](https://travis-ci.com/kylevedder/ServiceRobotControlStack)

## Requirements

 - ROS Melodic
 - Clone using `--recurse-submodules`
 - URG ROS Package (for deployment on real hardware)

## Setup

 - Install all tools and packages
   - Install [ROS Melodic](http://wiki.ros.org/melodic/Installation)
   - Run `./InstallPackages`
 - Setup commit hooks
   - Run `./ci/setup_hooks.sh` in the root of the repo
   
## Usage
 - All ROS nodes live in a single ROS package, `control_stack`
 - `nav_node` serves as the single navigation ROS node for both simulation and robot deployment
 - `ut_multirobot_sim` serves as a drop-in replacement for the real robot sensors, allowing for testing to be performed on the desktop
 - For more information, see the [wiki](https://github.com/kylevedder/ServiceRobotControlStack/wiki)

## License:

[MIT](../master/LICENSE)

