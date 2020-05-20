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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <fstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "config_reader/config_reader.h"
#include "cs/localization/particle_filter.h"
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/path_finding/rrt.h"
#include "cs/util/datastructures/circular_buffer.h"
#include "cs/util/icp.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

namespace params {
CONFIG_FLOAT(kDistanceFromMax, "od.kDistanceFromMax");
CONFIG_INT(laser_deadzone_left_min, "laser.laser_deadzone_left_min");
CONFIG_INT(laser_deadzone_left_max, "laser.laser_deadzone_left_max");
CONFIG_INT(laser_deadzone_right_min, "laser.laser_deadzone_right_min");
CONFIG_INT(laser_deadzone_right_max, "laser.laser_deadzone_right_max");
}  // namespace params

util::Pose UpdatePose(const util::Pose& pose, const util::Twist& twist) {
  float robot_delta_x = twist.tra.x();
  float robot_delta_y = 0;
  float delta_rot = 0;

  if (fabs(twist.rot) > kEpsilon) {
    const float rotation_radius = fabs(twist.tra.x()) / fabs(twist.rot);
    NP_CHECK(rotation_radius >= 0);
    const int translation_sign = math_util::Sign(twist.tra.x());
    const int rotation_sign = math_util::Sign(twist.rot);
    robot_delta_x =
        translation_sign * rotation_radius * std::sin(fabs(twist.rot));
    robot_delta_y =
        rotation_sign * rotation_radius * (1 - std::cos(fabs(twist.rot)));
    delta_rot = twist.rot;
  }

  // Robot frame deltas.

  NP_FINITE(robot_delta_x);
  NP_FINITE(robot_delta_y);
  NP_FINITE(delta_rot);

  const Eigen::Vector2f robot_delta(robot_delta_x, robot_delta_y);
  const Eigen::Vector2f world_delta =
      Eigen::Rotation2Df(pose.rot) * robot_delta;

  return {pose.tra + world_delta, math_util::AngleMod(pose.rot + delta_rot)};
}

struct PoseLaser {
  util::Pose pose;
  util::LaserScan laser;
  bool initialized;

  PoseLaser() : pose(0, 0, 0), laser(), initialized(false) {}

  PoseLaser(const util::Pose& pose, util::LaserScan& laser)
      : pose(pose), laser(laser), initialized(true) {}
};

struct TwistTime {
  util::Twist twist;
  ros::Time last_update;
  bool initialized;

  TwistTime() : twist(0, 0, 0), last_update(0), initialized(false) {}

  TwistTime(const util::Twist& twist, const ros::Time& last_update)
      : twist(twist), last_update(last_update), initialized(true) {}
};

struct CallbackWrapper {
  PoseLaser pose_laser_;
  TwistTime twist_time_;

  std::vector<std::pair<util::Pose, util::LaserScan>> poses_;

  CallbackWrapper() = default;

  static constexpr bool kUseICP = false;

  void LaserCallback(const sensor_msgs::LaserScan& msg) {
    util::LaserScan laser(msg);
    laser.ClearDataInIndexRange(params::CONFIG_laser_deadzone_right_min,
                                params::CONFIG_laser_deadzone_right_max);
    laser.ClearDataInIndexRange(params::CONFIG_laser_deadzone_left_min,
                                params::CONFIG_laser_deadzone_left_max);
    laser.ClearDataFilter([](const float& e) { return e > 0.5 && e < 3; });

    if (!pose_laser_.initialized) {
      pose_laser_ = {util::Pose(0, 0, 0), laser};
      return;
    }

    const util::Pose odom_estimated_pose =
        UpdatePose(pose_laser_.pose, twist_time_.twist);

    const util::Twist odom_estimated_ridged_transform(
        odom_estimated_pose.tra - pose_laser_.pose.tra,
        math_util::AngleMod(odom_estimated_pose.rot - pose_laser_.pose.rot));

    util::Twist blended_transform = odom_estimated_ridged_transform;

    if (kUseICP) {
      const auto res = util::solver::ICP(
          pose_laser_.laser, laser, odom_estimated_ridged_transform);
      const util::Twist& icp_estimated_ridged_transform = res.second;

      const float transform_diff = (icp_estimated_ridged_transform.tra -
                                    odom_estimated_ridged_transform.tra)
                                       .squaredNorm();

      blended_transform = (odom_estimated_ridged_transform * 0 +
                           icp_estimated_ridged_transform * 1);
      if (!res.first || !icp_estimated_ridged_transform.IsFinite() ||
          !blended_transform.IsFinite() || transform_diff > 10) {
        blended_transform = odom_estimated_ridged_transform;
      }
    }

    CHECK(blended_transform.IsFinite());
    const util::Pose est_pose =
        pose_laser_.pose +
        util::Pose(blended_transform.tra, blended_transform.rot);
    CHECK(est_pose.IsFinite());

    pose_laser_ = {est_pose, laser};
    twist_time_.twist = util::Twist(0, 0, 0);
    poses_.push_back({pose_laser_.pose, laser});
    CHECK(pose_laser_.pose.IsFinite());
    std::cout << poses_.size() << "\r";
    fflush(stdout);
  }

  void OdomCallback(const nav_msgs::Odometry& odom) {
    if (!twist_time_.initialized) {
      // Drops the first twist update as we don't know elapsed time.
      twist_time_ = {util::Twist(0, 0, 0), odom.header.stamp};
      return;
    }

    const util::Twist unscaled_delta_twist(odom.twist.twist);
    const float delta_t = static_cast<float>(
        (odom.header.stamp - twist_time_.last_update).toSec());
    const util::Twist scaled_delta_twist = unscaled_delta_twist * delta_t;
    const util::Twist moved_twist = twist_time_.twist + scaled_delta_twist;

    twist_time_ = {moved_twist, odom.header.stamp};
  }
};

std::tuple<std::string> GetArgs(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: [bag file]" << std::endl;
    exit(0);
  }
  return {argv[1]};
}

void TestUpdatePose() {
  util::Pose p(0, 0, 0);
  util::Twist t(kPi / 4, 0, 0);

  util::Pose out_p = UpdatePose(p, t);
  std::cout << out_p << std::endl;
  exit(0);
}

int main(int argc, char** argv) {
  std::cout << "Starting bag SLAM\n";
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/bag_nav_config.lua"});
  auto res = GetArgs(argc, argv);
  rosbag::Bag bag(std::get<0>(res));
  CallbackWrapper cw;

  //  TestUpdatePose();

  for (const auto& m : rosbag::View(bag)) {
    if (m.getTopic() == "/scan") {
      auto s = m.instantiate<sensor_msgs::LaserScan>();
      CHECK(s != nullptr);
      cw.LaserCallback(*s);
    }
    if (m.getTopic() == "/odom") {
      auto s = m.instantiate<nav_msgs::Odometry>();
      CHECK(s != nullptr);
      cw.OdomCallback(*s);
    }
  }

  std::cout << "Found " << cw.poses_.size() << " poses\n";

  ros::init(argc, argv, "bag_slam");
  ros::NodeHandle n;

  auto poses_pub = n.advertise<visualization_msgs::MarkerArray>("/poses", 1000);
  auto points_pub =
      n.advertise<visualization_msgs::MarkerArray>("/points", 1000);

  visualization_msgs::MarkerArray arr;
  visualization_msgs::MarkerArray arr_points;
  int i = 0;
  const float num_poses = static_cast<float>(cw.poses_.size());

  std::cout << "First pose: " << cw.poses_.front().first << std::endl;
  std::cout << "Last pose: " << cw.poses_.back().first << std::endl;

  for (const auto& pair : cw.poses_) {
    const util::Pose& p = pair.first;
    const auto points = pair.second.TransformPointsFrameSparse(p.ToAffine());
    CHECK(p.IsFinite());
    ++i;
    if (i % 10 == 0) {
      CHECK(p.IsFinite());
      CHECK(fabs(p.tra.x()) < 100) << p.tra.x();
      CHECK(fabs(p.tra.y()) < 100) << p.tra.y();
      CHECK(fabs(p.rot) < 10) << p.rot;
      visualization::DrawPose(p, "world_frame", "pose", 1, 0, 0, 1, &arr);
      visualization::DrawPoints(points,
                                "world_frame",
                                "points",
                                0,
                                (i / num_poses),
                                0,
                                1,
                                &arr_points);
    }
  }

  ros::Rate r(1);

  while (ros::ok()) {
    std::cout << "Publishing" << std::endl;
    poses_pub.publish(arr);
    points_pub.publish(arr_points);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
