// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <fstream>

#include "config_reader/config_reader.h"
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/particle_filter/particle_filter.h"
#include "cs/util/datastructures/circular_buffer.h"
#include "cs/util/math_util.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"

static constexpr bool kDebug = true;

namespace params {
CONFIG_STRING(kMap, "pf.kMap");
CONFIG_FLOAT(kInitX, "pf.kInitX");
CONFIG_FLOAT(kInitY, "pf.kInitY");
CONFIG_FLOAT(kInitTheta, "pf.kInitTheta");
CONFIG_FLOAT(kRobotRadius, "pf.kRobotRadius");
CONFIG_FLOAT(kCollisionRollout, "pf.kCollisionRollout");
CONFIG_FLOAT(kDesiredCommandX, "od.kDesiredCommandX");
CONFIG_FLOAT(kDesiredCommandRot, "od.kDesiredCommandRot");
CONFIG_FLOAT(kCommandScalar, "od.kCommandScalar");

// static constexpr auto kMap =
//    "./src/ServiceRobotControlStack/control_stack/maps/loop.map";
// static constexpr float kInitX = 4;
// static constexpr float kInitY = 0;
// static constexpr float kInitTheta = 0;
// static constexpr float kRobotRadius = 0.1;
// static constexpr float kCollisionRollout = 2;
// static constexpr float kDesiredCommandX = 0.2;
// static constexpr float kDesiredCommandRot = 0;
// static constexpr float kCommandScalar = 1;
}  // namespace params

static constexpr size_t kTimeBufferSize = 5;

struct CallbackWrapper {
  util::Map map_;
  cs::localization::ParticleFilter particle_filter_;
  cs::obstacle_avoidance::ObstacleDetector obstacle_detector_;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize>
      odom_times_buffer_;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize>
      laser_times_buffer_;
  tf::TransformBroadcaster br_;

  // Functionality pub/sub
  ros::Publisher velocity_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_sub_;

  // Debug pub/sub
  ros::Publisher particle_pub_;
  ros::Publisher map_pub_;
  ros::Publisher detected_walls_pub_;
  ros::Publisher robot_size_pub_;

  CallbackWrapper() = delete;

  CallbackWrapper(const util::Map& map, ros::NodeHandle* n)
      : map_(map),
        particle_filter_(map,
                         {params::kInitX, params::kInitY, params::kInitTheta}),
        obstacle_detector_(map) {
    velocity_pub_ = n->advertise<geometry_msgs::Twist>(
        constants::kCommandVelocityTopic, 10);
    laser_sub_ = n->subscribe(
        constants::kLaserTopic, 10, &CallbackWrapper::LaserCallback, this);
    odom_sub_ = n->subscribe(
        constants::kOdomTopic, 10, &CallbackWrapper::OdomCallback, this);

    if (kDebug) {
      particle_pub_ =
          n->advertise<visualization_msgs::MarkerArray>("particles", 10);
      map_pub_ = n->advertise<visualization_msgs::Marker>("map", 10);
      detected_walls_pub_ = n->advertise<visualization_msgs::MarkerArray>(
          "detected_obstacles", 10);
      robot_size_pub_ =
          n->advertise<visualization_msgs::Marker>("robot_size", 10);
    }
  }

  void LaserCallback(const sensor_msgs::LaserScan& msg) {
    const ros::Time& current_time = msg.header.stamp;
    laser_times_buffer_.push_back(current_time);
    const double mean_time_delta =
        (laser_times_buffer_.size() <= 1)
            ? kEpsilon
            : (laser_times_buffer_.back() - laser_times_buffer_.front())
                      .toSec() /
                  (laser_times_buffer_.size() - 1);
    NP_CHECK_VAL(mean_time_delta > 0, mean_time_delta);
    util::LaserScan laser(msg);
    particle_filter_.UpdateObservation(laser);
    const auto est_pose = particle_filter_.WeightedCentroid();
    obstacle_detector_.UpdateObservation(est_pose, laser, &detected_walls_pub_);
    ROS_INFO("Laser update. Est pose: (%f, %f), %f",
             est_pose.tra.x(),
             est_pose.tra.y(),
             est_pose.rot);
    const util::Twist commanded_velocity = CommandVelocity(
        {params::kDesiredCommandX, 0, params::kDesiredCommandRot},
        mean_time_delta);
    obstacle_detector_.UpdateCommand(commanded_velocity);
    PublishTransforms();
    if (kDebug) {
      particle_filter_.DrawParticles(&particle_pub_);
      robot_size_pub_.publish(visualization::MakeCylinder(est_pose.tra,
                                                          params::kRobotRadius,
                                                          3.0,
                                                          "map",
                                                          "robot_size",
                                                          0,
                                                          1,
                                                          0,
                                                          1));
    }
  }

  void OdomCallback(const nav_msgs::Odometry& msg) {
    const ros::Time& current_time = msg.header.stamp;
    ROS_INFO("Odom update");
    if (odom_times_buffer_.empty() ||
        odom_times_buffer_.back().toSec() >= current_time.toSec()) {
      odom_times_buffer_.push_back(current_time);
      return;
    }
    odom_times_buffer_.push_back(current_time);
    const util::Twist velocity(msg.twist.twist);
    const double mean_time_delta =
        (odom_times_buffer_.back() - odom_times_buffer_.front()).toSec() /
        (odom_times_buffer_.size() - 1);
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
    const util::Twist safe_cmd =
        obstacle_detector_.MakeCommandSafe(desired_command,
                                           time_delta,
                                           params::kCollisionRollout,
                                           params::kRobotRadius) *
        params::kCommandScalar;
    velocity_pub_.publish(safe_cmd.ToTwist());
    ROS_INFO("Command (%f, %f), %f sent",
             safe_cmd.tra.x(),
             safe_cmd.tra.y(),
             safe_cmd.rot);
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
    ROS_INFO("Transforms published!");
  }
};

int main(int argc, char** argv) {
  ROS_ERROR("Directory: %s", util::GetCurrentWorkingDirectory().c_str());
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/pf_sim_config.lua"});
  ros::init(argc, argv, "nav_node");

  ros::NodeHandle n;

  CallbackWrapper cw(util::Map(params::kMap), &n);

  ros::spin();

  return 0;
}
