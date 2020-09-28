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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include "config_reader/config_reader.h"

#include "cs/localization/particle_filter.h"
#include "cs/util/constants.h"
#include "cs/util/datastructures/circular_buffer.h"
#include "cs/util/laser_scan.h"
#include "cs/util/twist.h"
#include "cs/util/visualization.h"
#include "shared/util/timer.h"

namespace params {
CONFIG_STRING(map, "pf.map");
CONFIG_INTLIST(deadzones, "laser.deadzones");
CONFIG_VECTOR2F(laser_offset, "laser.center_offset");

CONFIG_BOOL(use_sim_ground_truth, "state_estimation.use_sim_ground_truth");
CONFIG_STRING(command_scaler, "cmd_scaler.command_scaler");
CONFIG_VECTOR3F(start_pose, "pf.start_pose");

CONFIG_STRING(map_tf_frame, "frames.map_tf_frame");
CONFIG_STRING(base_link_tf_frame, "frames.base_tf_frame");
CONFIG_STRING(laser_tf_frame, "frames.laser_tf_frame");

CONFIG_FLOAT(robot_radius, "pf.kRobotRadius");
CONFIG_FLOAT(safety_margin, "pf.kSafetyMargin");
}  // namespace params

struct PFUpdater {
  const util::vector_map::VectorMap map_;
  cs::localization::ParticleFilter pf_;
  ros::Publisher* particle_pub_;
  ros::Publisher* map_pub_;
  ros::Publisher* laser_pub_;
  tf::TransformBroadcaster br_;
  static constexpr size_t kTimeBufferSize = 5;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize> odom_times_;

  PFUpdater() = delete;
  PFUpdater(const std::string& map_path,
            const util::Pose& start_pose,
            ros::Publisher* particle_pub,
            ros::Publisher* map_pub,
            ros::Publisher* laser_pub)
      : map_(map_path),
        pf_(map_, start_pose),
        particle_pub_(particle_pub),
        map_pub_(map_pub),
        laser_pub_(laser_pub),
        br_() {}

  float GetOdomTimeDelta() const {
    const auto& b = odom_times_;
    if (b.size() <= 1) {
      return kEpsilon;
    }
    const double total_time_delta = (b.back() - b.front()).toSec();
    const double iterations = static_cast<double>(b.size() - 1);
    return static_cast<float>(total_time_delta / iterations);
  }

  void UpdateLaser(const sensor_msgs::LaserScan& msg) {
    util::LaserScan laser(msg);
    pf_.UpdateObservation(laser);

    PublishTransforms();

    // Republish laser with current timestamp to fix complaint about old
    // timestamp in bag file.
    laser.ros_laser_scan_.header.stamp = ros::Time::now();
    laser_pub_->publish(laser.ros_laser_scan_);
    pf_.DrawParticles(particle_pub_);
    map_pub_->publish(visualization::DrawWalls(map_.lines, "map", "map_ns"));
  }

  void UpdateOdom(const nav_msgs::Odometry& msg) {
    util::Twist odom(msg.twist.twist);
    // Convert odom from velocity to delta over timestep.
    odom = odom * GetOdomTimeDelta();
    pf_.UpdateOdom(odom.tra.x(), odom.rot);
    odom_times_.push_back(msg.header.stamp);
  }

  void PublishTransforms() {
    tf::Transform laser_transform = tf::Transform::getIdentity();
    laser_transform.setOrigin({-params::CONFIG_laser_offset.x(),
                               -params::CONFIG_laser_offset.y(),
                               0});
    br_.sendTransform(tf::StampedTransform(laser_transform,
                                           ros::Time::now(),
                                           params::CONFIG_laser_tf_frame,
                                           params::CONFIG_base_link_tf_frame));

    const auto est_pose = pf_.WeightedCentroid();
    NP_FINITE_VEC(est_pose.tra);
    NP_FINITE(est_pose.rot);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(est_pose.tra.x(), est_pose.tra.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, est_pose.rot);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform.inverse(),
                                           ros::Time::now(),
                                           params::CONFIG_base_link_tf_frame,
                                           params::CONFIG_map_tf_frame));
  }
};

int main(int argc, char** argv) {
  const std::string config_file =
      "src/control_stack/config/getting_started_config.lua";

  ROS_INFO("Using config file: %s", config_file.c_str());

  config_reader::ConfigReader reader({config_file});
  ros::init(argc, argv, "getting_started");
  ros::NodeHandle n;

  ros::Publisher particle_pub =
      n.advertise<visualization_msgs::MarkerArray>("/particles", 5);
  ros::Publisher map_pub = n.advertise<visualization_msgs::Marker>("/map", 5);
  ros::Publisher laser_pub =
      n.advertise<sensor_msgs::LaserScan>("/laser_repub", 5);

  PFUpdater pf_updater(
      params::CONFIG_map, {0, 0, 0}, &particle_pub, &map_pub, &laser_pub);

  ros::Subscriber laser_sub =
      n.subscribe("/scan", 5, &PFUpdater::UpdateLaser, &pf_updater);
  ros::Subscriber odom_sub =
      n.subscribe("/odom", 5, &PFUpdater::UpdateOdom, &pf_updater);

  ros::spin();
  return 0;
}
