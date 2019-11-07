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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>

#include "config_reader/config_reader.h"
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/particle_filter/particle_filter.h"
#include "cs/util/crash_handling.h"
#include "cs/util/math_util.h"
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
}  // namespace params

struct CallbackWrapper {
  util::Map map_;
  cs::localization::ParticleFilter particle_filter_;
  cs::obstacle_avoidance::ObstacleDetector obstacle_detector_;
  ros::Time last_odom_update_;

  // Functionality pub/sub
  ros::Publisher velocity_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_sub_;

  // Debug pub/sub
  ros::Publisher particle_pub_;

  CallbackWrapper() = delete;

  CallbackWrapper(const util::Map& map, ros::NodeHandle* n)
      : map_(map),
        particle_filter_(map,
                         {params::kInitX, params::kInitY, params::kInitTheta}),
        obstacle_detector_(map),
        last_odom_update_(0) {
    velocity_pub_ =
        n->advertise<geometry_msgs::Twist>(kCommandVelocityTopic, 10);
    laser_sub_ =
        n->subscribe(kLaserTopic, 10, &CallbackWrapper::LaserCallback, this);
    odom_sub_ =
        n->subscribe(kOdomTopic, 10, &CallbackWrapper::OdomCallback, this);

    if (kDebug) {
      particle_pub_ =
          n->advertise<visualization_msgs::MarkerArray>("particles", 10);
    }
  }

  void LaserCallback(const sensor_msgs::LaserScan& msg) {
    util::LaserScan laser(msg);
    particle_filter_.UpdateObservation(laser);
    const auto est_pose = particle_filter_.WeightedCentroid();
    obstacle_detector_.UpdateObservation(est_pose, laser);
    ROS_INFO("Laser update. Est pose: (%f, %f), %f", est_pose.tra.x(),
             est_pose.tra.y(), est_pose.rot);
    CommandVelocity();
    if (kDebug) {
      particle_filter_.DrawParticles(&particle_pub_);
    }
  }

  void OdomCallback(const nav_msgs::Odometry& msg) {
    const ros::Time current_time = msg.header.stamp;
    ROS_INFO("Odom update");
    if (last_odom_update_ == ros::Time(0) ||
        last_odom_update_.toSec() >= current_time.toSec()) {
      ROS_INFO("Last time: %f", last_odom_update_.toSec());
      ROS_INFO("Current time: %f", current_time.toSec());
      last_odom_update_ = current_time;
      ROS_INFO("Odom exit early");
      return;
    }
    const util::Pose velocity(msg.twist.twist);
    const float time_delta = (current_time - last_odom_update_).toSec();
    const util::Pose delta = velocity * time_delta;
    particle_filter_.UpdateOdom(delta.tra.x(), delta.rot);
    if (kDebug) {
      particle_filter_.DrawParticles(&particle_pub_);
    }
    last_odom_update_ = current_time;
    obstacle_detector_.UpdateOdom(particle_filter_.WeightedCentroid(),
                                  velocity);
  }

  void CommandVelocity() {
    const util::Pose desired_command(1, 0, 0);
    const util::Pose safe_cmd = obstacle_detector_.MakeCommandSafe(
        desired_command, 2, params::kRobotRadius);
    velocity_pub_.publish(safe_cmd.ToTwist());
    ROS_INFO("Command (%f, %f), %f sent", safe_cmd.tra.x(), safe_cmd.tra.y(),
             safe_cmd.rot);
  }
};

int main(int argc, char** argv) {
  util::PrintCurrentWorkingDirectory();
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/pf_sim_config.lua"});
  ros::init(argc, argv, "nav_node");

  ros::NodeHandle n;

  CallbackWrapper cw(util::Map(params::kMap), &n);

  ros::spin();

  return 0;
}
