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
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/path_finding/rrt.h"
#include "cs/util/datastructures/circular_buffer.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

static constexpr bool kDebug = true;

namespace params {
CONFIG_FLOAT(kRobotRadius, "pf.kRobotRadius");
CONFIG_FLOAT(kSafetyMargin, "pf.kSafetyMargin");
CONFIG_FLOAT(kCollisionRollout, "pf.kCollisionRollout");
CONFIG_INT(kTranslateCommandSign, "od.kTranslateCommandSign");

CONFIG_FLOAT(rotation_drive_threshold, "control.rotation_drive_threshold");
CONFIG_FLOAT(rotation_p, "control.rotation_p");
CONFIG_FLOAT(translation_p, "control.translation_p");
CONFIG_FLOAT(goal_deadzone_tra, "control.goal_deadzone_tra");
CONFIG_INT(laser_deadzone_left_min, "laser.laser_deadzone_left_min");
CONFIG_INT(laser_deadzone_left_max, "laser.laser_deadzone_left_max");
CONFIG_INT(laser_deadzone_right_min, "laser.laser_deadzone_right_min");
CONFIG_INT(laser_deadzone_right_max, "laser.laser_deadzone_right_max");
}  // namespace params

static constexpr size_t kTimeBufferSize = 5;

struct CallbackWrapper {
  util::Map map_;
  cs::obstacle_avoidance::ObstacleDetector obstacle_detector_;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize>
      odom_times_buffer_;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize>
      laser_times_buffer_;
  tf::TransformBroadcaster br_;
  ros::Time current_command_time_;
  util::Twist current_command_;

  // Functionality pub/sub
  ros::Publisher velocity_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber teleop_sub_;

  // Debug pub/sub
  ros::Publisher modified_laser_pub_;
  ros::Publisher detected_walls_pub_;
  ros::Publisher robot_size_pub_;

  CallbackWrapper() = delete;

  explicit CallbackWrapper(ros::NodeHandle* n)
      : map_(),
        obstacle_detector_(map_),
        current_command_time_(ros::Time::now()),
        current_command_() {
    velocity_pub_ = n->advertise<geometry_msgs::Twist>(
        constants::kCommandVelocityTopic, 10);
    laser_sub_ = n->subscribe(
        constants::kLaserTopic, 10, &CallbackWrapper::LaserCallback, this);
    odom_sub_ = n->subscribe(
        constants::kOdomTopic, 10, &CallbackWrapper::OdomCallback, this);
    teleop_sub_ = n->subscribe(
        constants::kTeleopTopic, 10, &CallbackWrapper::TeleopCallback, this);

    if (kDebug) {
      modified_laser_pub_ =
          n->advertise<sensor_msgs::LaserScan>("scan_modified", 10);
      detected_walls_pub_ = n->advertise<visualization_msgs::MarkerArray>(
          "detected_obstacles", 10);
      robot_size_pub_ =
          n->advertise<visualization_msgs::Marker>("robot_size", 10);
    }
  }

  void TeleopCallback(const geometry_msgs::Twist& msg) {
    auto t = util::Twist({msg.linear.x, 0}, msg.angular.z);
    std::cout << "Current command: " << t << std::endl;
    current_command_ = t;
    current_command_time_ = ros::Time::now();
  }

  void LaserCallback(const sensor_msgs::LaserScan& msg) {
    util::LaserScan laser(msg);
    laser.ClearDataInIndexRange(params::CONFIG_laser_deadzone_right_min,
                                params::CONFIG_laser_deadzone_right_max);
    laser.ClearDataInIndexRange(params::CONFIG_laser_deadzone_left_min,
                                params::CONFIG_laser_deadzone_left_max);
    if (kDebug) {
      modified_laser_pub_.publish(laser.ros_laser_scan_);
    }
    const ros::Time& current_time = msg.header.stamp;
    laser_times_buffer_.push_back(current_time);
    const double mean_time_delta =
        (laser_times_buffer_.size() <= 1)
            ? kEpsilon
            : (laser_times_buffer_.back() - laser_times_buffer_.front())
                      .toSec() /
                  static_cast<double>(laser_times_buffer_.size() - 1);
    NP_CHECK_VAL(mean_time_delta > 0, mean_time_delta);
    obstacle_detector_.UpdateObservation(
        {0, 0, 0}, laser, &detected_walls_pub_);
    const util::Twist commanded_velocity = CommandVelocity(
        GetDesiredCommand(), static_cast<float>(mean_time_delta));
    obstacle_detector_.UpdateCommand(commanded_velocity);
    PublishTransforms();
    if (kDebug) {
      robot_size_pub_.publish(
          visualization::MakeCylinder({0, 0},
                                      params::CONFIG_kRobotRadius,
                                      3.0,
                                      "map",
                                      "robot_size",
                                      0,
                                      1,
                                      0,
                                      1));
    }
  }

  util::Twist TransformTwistUsingSign(util::Twist twist) {
    twist.tra *= params::CONFIG_kTranslateCommandSign;
    return twist;
  }

  void OdomCallback(const nav_msgs::Odometry& msg) {
    const ros::Time& current_time = msg.header.stamp;
    //    ROS_INFO("Odom update");
    if (odom_times_buffer_.empty() ||
        odom_times_buffer_.back().toSec() >= current_time.toSec()) {
      odom_times_buffer_.push_back(current_time);
      return;
    }
    odom_times_buffer_.push_back(current_time);
    //    const util::Twist velocity =
    //        TransformTwistUsingSign(util::Twist(msg.twist.twist));
    //    const double mean_time_delta =
    //        (odom_times_buffer_.back() - odom_times_buffer_.front()).toSec() /
    //        static_cast<double>(odom_times_buffer_.size() - 1);
    //    NP_CHECK(mean_time_delta > 0);
    //    const util::Twist delta = velocity *
    //    static_cast<float>(mean_time_delta); std::cout << delta << std::endl;
    //    particle_filter_.UpdateOdom(delta.tra.x(), delta.rot);
  }

  util::Twist GetDesiredCommand() const {
    if ((ros::Time::now() - current_command_time_).toSec() > 1.5) {
      return util::Twist(0, 0, 0);
    }
    return current_command_;
  }

  util::Twist CommandVelocity(const util::Twist& desired_command,
                              const float& time_delta) {
    static const util::Twist kZeroTwist(0, 0, 0);
    if (desired_command == kZeroTwist) {
      return desired_command;
    }
    const util::Twist safe_cmd =
        obstacle_detector_.MakeCommandSafe(desired_command,
                                           time_delta,
                                           params::CONFIG_kCollisionRollout,
                                           params::CONFIG_kRobotRadius,
                                           params::CONFIG_kSafetyMargin);
    const util::Twist sent_cmd = TransformTwistUsingSign(safe_cmd);
    velocity_pub_.publish(sent_cmd.ToTwist());
    return safe_cmd;
  }

  void PublishTransforms() {
    br_.sendTransform(tf::StampedTransform(
        tf::Transform::getIdentity(), ros::Time::now(), "laser", "base_link"));
  }
};

int main(int argc, char** argv) {
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/teleop_config.lua"});
  ros::init(argc, argv, "teleop_node");

  ros::NodeHandle n;

  CallbackWrapper cw(&n);

  std::cout << "Teleop node started!\n";
  ros::spin();

  return 0;
}
