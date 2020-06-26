// Copyright 2019 - 2020 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ========================================================================

#include <ros/ros.h>

#include "config_reader/config_reader.h"
#include "cs/main/callback_wrapper.h"
#include "cs/main/debug_pub_wrapper.h"
#include "cs/main/state_machine.h"

namespace params {
CONFIG_STRING(map, "pf.map");
}

int main(int argc, char** argv) {
  std::string config_file = "src/control_stack/config/nav_config.lua";
  if (argc == 2) {
    config_file = argv[1];
  }
  ROS_INFO("Using config file: %s", config_file.c_str());
  config_reader::ConfigReader reader({config_file});
  ros::init(argc, argv, "nav_node");
  ros::NodeHandle n;

  cs::main::DebugPubWrapper dpw(&n);
  cs::main::StateMachine state_machine(&dpw, &n);

  ros::Subscriber laser_sub = n.subscribe(constants::kLaserTopic,
                                          1,
                                          &cs::main::StateMachine::UpdateLaser,
                                          &state_machine);
  ros::Subscriber odom_sub = n.subscribe(constants::kOdomTopic,
                                         1,
                                         &cs::main::StateMachine::UpdateOdom,
                                         &state_machine);
  ros::Publisher command_pub =
      n.advertise<geometry_msgs::Twist>(constants::kCommandVelocityTopic, 1);

  RateLoop rate(40.0);
  while (ros::ok()) {
    ros::spinOnce();
    command_pub.publish(state_machine.ExecuteController().ToTwist());
    rate.Sleep();
  }
  return 0;
}
