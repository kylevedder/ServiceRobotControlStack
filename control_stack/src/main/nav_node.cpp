// Copyright 2019 kvedder@seas.upenn.edu
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
CONFIG_STRING(kMap, "pf.kMap");
CONFIG_FLOAT(kInitX, "pf.kInitX");
CONFIG_FLOAT(kInitY, "pf.kInitY");
CONFIG_FLOAT(kInitTheta, "pf.kInitTheta");
CONFIG_FLOAT(kGoalX, "pf.kGoalX");
CONFIG_FLOAT(kGoalY, "pf.kGoalY");
CONFIG_FLOAT(kRobotRadius, "pf.kRobotRadius");
CONFIG_FLOAT(kSafetyMargin, "pf.kSafetyMargin");
CONFIG_FLOAT(kCollisionRollout, "pf.kCollisionRollout");
CONFIG_INT(kTranslateCommandSign, "od.kTranslateCommandSign");

CONFIG_FLOAT(rotation_drive_threshold, "control.rotation_drive_threshold");
CONFIG_FLOAT(rotation_p, "control.rotation_p");
CONFIG_FLOAT(translation_p, "control.translation_p");
CONFIG_FLOAT(goal_deadzone_tra, "control.goal_deadzone_tra");
}  // namespace params

static constexpr size_t kTimeBufferSize = 5;

struct CallbackWrapper {
  util::Map map_;
  cs::localization::ParticleFilter particle_filter_;
  cs::obstacle_avoidance::ObstacleDetector obstacle_detector_;
  cs::path_finding::RRT path_finder_;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize>
      odom_times_buffer_;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize>
      laser_times_buffer_;
  tf::TransformBroadcaster br_;
  Eigen::Vector2f current_goal_;

  // Functionality pub/sub
  ros::Publisher velocity_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber goal_sub_;

  // Debug pub/sub
  ros::Publisher particle_pub_;
  ros::Publisher map_pub_;
  ros::Publisher detected_walls_pub_;
  ros::Publisher robot_size_pub_;
  ros::Publisher robot_path_pub_;
  ros::Publisher rrt_tree_pub_;

  CallbackWrapper() = delete;

  CallbackWrapper(const std::string& map_file, ros::NodeHandle* n)
      : map_(map_file),
        particle_filter_(map_,
                         {params::CONFIG_kInitX,
                          params::CONFIG_kInitY,
                          params::CONFIG_kInitTheta}),
        obstacle_detector_(map_),
        path_finder_(
            map_, params::CONFIG_kRobotRadius, params::CONFIG_kSafetyMargin),
        current_goal_(params::CONFIG_kGoalX, params::CONFIG_kGoalY) {
    velocity_pub_ = n->advertise<geometry_msgs::Twist>(
        constants::kCommandVelocityTopic, 10);
    laser_sub_ = n->subscribe(
        constants::kLaserTopic, 10, &CallbackWrapper::LaserCallback, this);
    odom_sub_ = n->subscribe(
        constants::kOdomTopic, 10, &CallbackWrapper::OdomCallback, this);
    goal_sub_ = n->subscribe(
        constants::kGoalTopic, 10, &CallbackWrapper::GoalCallback, this);

    if (kDebug) {
      particle_pub_ =
          n->advertise<visualization_msgs::MarkerArray>("particles", 10);
      map_pub_ = n->advertise<visualization_msgs::Marker>("map", 10);
      detected_walls_pub_ = n->advertise<visualization_msgs::MarkerArray>(
          "detected_obstacles", 10);
      robot_size_pub_ =
          n->advertise<visualization_msgs::Marker>("robot_size", 10);
      robot_path_pub_ =
          n->advertise<visualization_msgs::Marker>("robot_path", 10);
      rrt_tree_pub_ = n->advertise<visualization_msgs::Marker>("rrt_tree", 10);
    }
  }

  static util::Twist DriveToWaypoint(const util::Pose& pose,
                                     const Eigen::Vector2f& waypoint) {
    const auto robot_heading = geometry::Heading(pose.rot);
    const Eigen::Vector2f waypoint_delta = (waypoint - pose.tra);
    const float distance_to_goal_sq = waypoint_delta.squaredNorm();
    NP_FINITE(distance_to_goal_sq);
    if (distance_to_goal_sq < Sq(params::CONFIG_goal_deadzone_tra)) {
      return {0, 0, 0};
    }

    const auto waypoint_heading = geometry::GetNormalizedOrZero(waypoint_delta);
    NP_FINITE_2F(waypoint_heading);
    const int angle_direction =
        math_util::Sign(geometry::Cross(robot_heading, waypoint_heading));
    NP_FINITE(angle_direction);
    const float angle = std::acos(robot_heading.dot(waypoint_heading));
    NP_FINITE(angle);
    NP_CHECK_VAL(angle >= 0 && angle <= kPi + kEpsilon, angle);
    float x = 0;
    if (angle < params::CONFIG_rotation_drive_threshold) {
      x = std::sqrt(distance_to_goal_sq);
      x *= math_util::Sign(waypoint_heading.dot(robot_heading));
    }
    return {x * params::CONFIG_translation_p,
            0,
            angle * angle_direction * params::CONFIG_rotation_p};
  }

  void GoalCallback(const geometry_msgs::Pose2D& msg) {
    current_goal_ = {msg.x, msg.y};
  }

  void LaserCallback(const sensor_msgs::LaserScan& msg) {
    const ros::Time& current_time = msg.header.stamp;
    laser_times_buffer_.push_back(current_time);
    const double mean_time_delta =
        (laser_times_buffer_.size() <= 1)
            ? kEpsilon
            : (laser_times_buffer_.back() - laser_times_buffer_.front())
                      .toSec() /
                  static_cast<double>(laser_times_buffer_.size() - 1);
    NP_CHECK_VAL(mean_time_delta > 0, mean_time_delta);
    util::LaserScan laser(msg);
    particle_filter_.UpdateObservation(laser);
    const auto est_pose = particle_filter_.WeightedCentroid();
    obstacle_detector_.UpdateObservation(est_pose, laser, &detected_walls_pub_);
    //    ROS_INFO("Laser update. Est pose: (%f, %f), %f",
    //             est_pose.tra.x(),
    //             est_pose.tra.y(),
    //             est_pose.rot);
    const auto& dynamic_map = obstacle_detector_.GetDynamicMap();
    const auto path = path_finder_.FindPath(
        dynamic_map, est_pose.tra, current_goal_, &rrt_tree_pub_);
    robot_path_pub_.publish(visualization::DrawPath(path, "map", "path"));
    util::Twist desired_command(0, 0, 0);
    if (path.IsValid()) {
      desired_command = DriveToWaypoint(est_pose, path.waypoints[1]);
      //      ROS_INFO("Waypoint drive: (%f, %f,) %f",
      //               desired_command.tra.x(),
      //               desired_command.tra.y(),
      //               desired_command.rot);
    }
    //    ROS_INFO("Desired command: (%f, %f,) %f",
    //             desired_command.tra.x(),
    //             desired_command.tra.y(),
    //             desired_command.rot);
    const util::Twist commanded_velocity =
        CommandVelocity(desired_command, static_cast<float>(mean_time_delta));
    obstacle_detector_.UpdateCommand(commanded_velocity);
    PublishTransforms();
    if (kDebug) {
      particle_filter_.DrawParticles(&particle_pub_);
      robot_size_pub_.publish(
          visualization::MakeCylinder(est_pose.tra,
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
    const util::Twist velocity =
        TransformTwistUsingSign(util::Twist(msg.twist.twist));
    const double mean_time_delta =
        (odom_times_buffer_.back() - odom_times_buffer_.front()).toSec() /
        static_cast<double>(odom_times_buffer_.size() - 1);
    NP_CHECK(mean_time_delta > 0);
    const util::Twist delta = velocity * static_cast<float>(mean_time_delta);
    particle_filter_.UpdateOdom(delta.tra.x(), delta.rot);
    if (kDebug) {
      particle_filter_.DrawParticles(&particle_pub_);
      map_pub_.publish(map_.ToMarker());
    }
    obstacle_detector_.UpdateOdom(particle_filter_.WeightedCentroid(),
                                  velocity);
  }

  util::Twist CommandVelocity(const util::Twist& desired_command,
                              const float& time_delta) {
    //    ROS_INFO("Command (%f, %f), %f desired for delta t: %f",
    //             desired_command.tra.x(),
    //             desired_command.tra.y(),
    //             desired_command.rot,
    //             time_delta);
    const util::Twist safe_cmd =
        obstacle_detector_.MakeCommandSafe(desired_command,
                                           time_delta,
                                           params::CONFIG_kCollisionRollout,
                                           params::CONFIG_kRobotRadius,
                                           params::CONFIG_kSafetyMargin);
    const util::Twist sent_cmd = TransformTwistUsingSign(safe_cmd);
    velocity_pub_.publish(sent_cmd.ToTwist());
    //    ROS_INFO("Command (%f, %f), %f safe",
    //             safe_cmd.tra.x(),
    //             safe_cmd.tra.y(),
    //             safe_cmd.rot);
    //    ROS_INFO("Command (%f, %f), %f sent",
    //             sent_cmd.tra.x(),
    //             sent_cmd.tra.y(),
    //             sent_cmd.rot);
    return safe_cmd;
  }

  void PublishTransforms() {
    br_.sendTransform(tf::StampedTransform(
        tf::Transform::getIdentity(), ros::Time::now(), "laser", "base_link"));

    const util::Pose current_pose = particle_filter_.WeightedCentroid();
    NP_FINITE_2F(current_pose.tra);
    NP_FINITE(current_pose.rot);
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(current_pose.tra.x(), current_pose.tra.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, current_pose.rot);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(
        transform.inverse(), ros::Time::now(), "base_link", "map"));
  }
};

int main(int argc, char** argv) {
  ROS_ERROR("Directory: %s", util::GetCurrentWorkingDirectory().c_str());
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/nav_config.lua"});
  ros::init(argc, argv, "nav_node");

  ros::NodeHandle n;

  CallbackWrapper cw(params::CONFIG_kMap, &n);

  ros::spin();

  return 0;
}
