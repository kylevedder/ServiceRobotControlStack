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

#include "cs/particle_filter/particle_filter.h"
#include "cs/util/crash_handling.h"
#include "cs/util/math_util.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"

#include "config_reader/config_reader.h"

struct ParticleFilterWrapper {
  util::Map map;
  cs::localization::ParticleFilter particle_filter;

  double last_odom_update_;

  ros::Publisher particle_pub;
  ros::Publisher sampled_laser_pub;
  ros::Publisher grid_belief_pub;
  ros::Publisher reference_pub;
  ros::Publisher map_pub;

  tf::TransformBroadcaster br;

  ParticleFilterWrapper() = delete;
  ParticleFilterWrapper(const util::Map& map, ros::NodeHandle* n)
      : map(map), particle_filter(map), last_odom_update_(0) {
    particle_pub =
        n->advertise<visualization_msgs::MarkerArray>("particles", 10);
    sampled_laser_pub =
        n->advertise<sensor_msgs::LaserScan>("sampled_laser", 100);
    grid_belief_pub =
        n->advertise<visualization_msgs::MarkerArray>("grid_belief", 10);
    reference_pub =
        n->advertise<visualization_msgs::MarkerArray>("reference", 10);
    map_pub = n->advertise<visualization_msgs::Marker>("map", 10);
  }

  void GridSearchBelief(const util::LaserScan& laser) {
    visualization_msgs::MarkerArray arr;
    float max_score = 0;
    static constexpr float kXMin = -0.75;
    static constexpr float kXMax = 0.75;
    static constexpr float kXDel = 0.1;
    static constexpr float kYMin = -0.75;
    static constexpr float kYMax = 0.75;
    static constexpr float kYDel = 0.1;
    static constexpr float kThetaMin = -kPi / 2;
    static constexpr float kThetaMax = kPi / 2 + kEpsilon;
    static constexpr float kThetaDel = kPi / 8;

    const auto& wc = particle_filter.WeightedCentroid();

    auto make_position = [wc](const float x, const float y,
                              const float theta) -> util::Pose {
      const Eigen::Vector2f offset_vector(x, y);
      return util::Pose(wc.tra + Eigen::Rotation2Df(wc.rot) * offset_vector,
                        theta + wc.rot);
    };

    for (float x = kXMin; x <= kXMax; x += kXDel) {
      for (float y = kYMin; y <= kYMax; y += kYDel) {
        for (float theta = kThetaMin; theta <= kThetaMax; theta += kThetaDel) {
          const util::Pose current_pose = make_position(x, y, theta);
          const float score =
              particle_filter.ScoreObservation(current_pose, laser);
          max_score = std::max(max_score, score);
        }
      }
    }

    for (float x = kXMin; x <= kXMax; x += kXDel) {
      for (float y = kYMin; y <= kYMax; y += kYDel) {
        for (float theta = kThetaMin; theta <= kThetaMax; theta += kThetaDel) {
          const Eigen::Vector2f offset_vector(x, y);
          const util::Pose current_pose = make_position(x, y, theta);
          const float score =
              particle_filter.ScoreObservation(current_pose, laser);
          const float red = score / max_score;
          visualization::DrawPose(current_pose, "map", "grid_search", red, 0, 0,
                                  1, &arr, theta);
        }
      }
    }

    ROS_INFO("Published %zu grid elements", arr.markers.size());
    grid_belief_pub.publish(arr);

    visualization_msgs::MarkerArray ref_arr;
    const Eigen::Vector2f offset_tra(1, 0.5);
    const float rotation = 0;
    const util::Pose base_link_reference(offset_tra, rotation);
    visualization::DrawPose(base_link_reference, "base_link", "reference", 0, 0,
                            0, 1, &ref_arr);

    const util::Pose global_reference(
        wc.tra + Eigen::Rotation2Df(wc.rot) * offset_tra, rotation + wc.rot);
    visualization::DrawPose(global_reference, "map", "reference", 1, 1, 1, 1,
                            &ref_arr, 0.1);
    ref_arr.markers.push_back(visualization::LaserToLineList(
        laser, wc, map, "map", "obs", 1, 0, 0, 1));
    reference_pub.publish(ref_arr);
  }

  void DrawMap() {
    map_pub.publish(map.ToMarker());
    tf::Transform transform;
    const util::Pose current_pose = particle_filter.WeightedCentroid();
    transform.setOrigin(
        tf::Vector3(current_pose.tra.x(), current_pose.tra.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, current_pose.rot);
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "map", "laser"));
  }

  void LaserCallback(const sensor_msgs::LaserScan& msg) {
    util::LaserScan laser(msg);
    particle_filter.UpdateObservation(laser, &sampled_laser_pub);
    particle_filter.DrawParticles(&particle_pub);
    DrawMap();

    // GridSearchBelief(laser);
  }

  void OdomCallback(const nav_msgs::Odometry& msg) {
    const double current_time = msg.header.stamp.toSec();
    if (last_odom_update_ == 0 || last_odom_update_ >= current_time) {
      last_odom_update_ = current_time;
      return;
    }
    util::Pose odom(msg.twist.twist);
    odom *= (current_time - last_odom_update_);
    particle_filter.UpdateOdom(odom.tra.x(), odom.rot);
    particle_filter.DrawParticles(&particle_pub);
    last_odom_update_ = current_time;
  }
};

int main(int argc, char** argv) {
  CONFIG_STRING(kMap, "pf.kMap");
  CONFIG_FLOAT(kInitX, "pf.kInitX");
  CONFIG_FLOAT(kInitY, "pf.kInitY");
  CONFIG_FLOAT(kInitTheta, "pf.kInitTheta");

  util::PrintCurrentWorkingDirectory();
  config_reader::ConfigReader reader(
      {"src/ParticleFilterCpp/particle_filter/config/"
       "pf_production_config.lua"});
  ros::init(argc, argv, "particle_filter", ros::init_options::NoSigintHandler);

  if (signal(SIGINT, util::crash::FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGINT" << std::endl;
    exit(-1);
  }
  if (signal(SIGSEGV, util::crash::FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGSEGV" << std::endl;
    exit(-1);
  }
  if (signal(SIGABRT, util::crash::FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGABRT" << std::endl;
    exit(-1);
  }

  ros::NodeHandle n;

  ParticleFilterWrapper wrapper(util::Map(kMap), &n);
  wrapper.particle_filter.InitalizePose({{kInitX, kInitY}, kInitTheta});

  ros::Subscriber laser_sub =
      n.subscribe("/scan", 10, &ParticleFilterWrapper::LaserCallback, &wrapper);
  ros::Subscriber odom_sub =
      n.subscribe("/odom", 10, &ParticleFilterWrapper::OdomCallback, &wrapper);

  ros::spin();

  return 0;
}
