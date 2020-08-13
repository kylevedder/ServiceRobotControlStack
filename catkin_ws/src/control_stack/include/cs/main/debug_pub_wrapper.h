#pragma once
// Copyright 2020 kvedder@seas.upenn.edu
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

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <fstream>
#include <string>

#include "config_reader/config_reader.h"
#include "cs/localization/particle_filter.h"
#include "cs/motion_planning/command_scaler.h"
#include "cs/motion_planning/identity_command_scaler.h"
#include "cs/motion_planning/pid.h"
#include "cs/motion_planning/turtlebot_command_scaler.h"
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/path_finding/astar.h"
#include "cs/path_finding/global_path_finder.h"
#include "cs/state_estimation/pf_state_estimator.h"
#include "cs/state_estimation/sim_state_estimator.h"
#include "cs/state_estimation/state_estimator.h"
#include "cs/util/physics.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

namespace cs {
namespace main {

struct DebugPubWrapper {
  ros::Publisher position_pub_;
  ros::Publisher modified_laser_pub_;
  ros::Publisher particle_pub_;
  ros::Publisher map_pub_;
  ros::Publisher detected_walls_pub_;
  ros::Publisher robot_size_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher robot_path_pub_;
  ros::Publisher colliding_point_pub_;

  DebugPubWrapper() = delete;

  //  ,
  //  void (T::*laser_callback)(const sensor_msgs::LaserScan&),
  //  void (T::*odom_callback)(const nav_msgs::Odometry&),
  //      T* obj
  explicit DebugPubWrapper(ros::NodeHandle* n,
                           const std::string& pub_sub_prefix) {
    position_pub_ = n->advertise<geometry_msgs::Twist>(
        pub_sub_prefix + constants::kPositionTopic, 1);
    modified_laser_pub_ = n->advertise<sensor_msgs::LaserScan>(
        pub_sub_prefix + "/scan_modified", 10);
    particle_pub_ = n->advertise<visualization_msgs::MarkerArray>(
        pub_sub_prefix + "/particles", 10);
    map_pub_ = n->advertise<visualization_msgs::Marker>(
        pub_sub_prefix + "/robot_map", 10);
    detected_walls_pub_ = n->advertise<visualization_msgs::MarkerArray>(
        pub_sub_prefix + "/detected_obstacles", 10);
    robot_size_pub_ = n->advertise<visualization_msgs::Marker>(
        pub_sub_prefix + "/robot_size", 10);
    goal_pub_ = n->advertise<visualization_msgs::MarkerArray>(
        pub_sub_prefix + "/goal", 10);
    robot_path_pub_ = n->advertise<visualization_msgs::Marker>(
        pub_sub_prefix + "/robot_path", 10);
    colliding_point_pub_ = n->advertise<visualization_msgs::Marker>(
        pub_sub_prefix + "/colliding_point", 10);
  }
};

}  // namespace main
}  // namespace cs
