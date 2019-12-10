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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "cs/util/constants.h"
#include "cs/util/geometry.h"
#include "cs/util/map.h"
#include "cs/util/math_util.h"
#include "cs/util/pose.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"

#include "config_reader/config_reader.h"

namespace sim {
CONFIG_FLOAT(kLaserStdDev, "sim.kLaserStdDev");
CONFIG_FLOAT(kArcExecStdDev, "sim.kArcExecStdDev");
CONFIG_FLOAT(kArcReadStdDev, "sim.kArcReadStdDev");
CONFIG_FLOAT(kRotateExecStdDev, "sim.kRotateExecStdDev");
CONFIG_FLOAT(kRotateReadStdDev, "sim.kRotateReadStdDev");
CONFIG_FLOAT(kStartPositionX, "sim.kStartPositionX");
CONFIG_FLOAT(kStartPositionY, "sim.kStartPositionY");
CONFIG_FLOAT(kStartPositionTheta, "sim.kStartPositionTheta");
CONFIG_STRING(kMap, "sim.kMap");

CONFIG_FLOAT(laser_min_angle, "sim.laser.min_angle");
CONFIG_FLOAT(laser_max_angle, "sim.laser.max_angle");
CONFIG_INT(laser_num_readings, "sim.laser.num_readings");
CONFIG_FLOAT(laser_angle_delta, "sim.laser.angle_delta");
CONFIG_FLOAT(laser_min_reading, "sim.laser.min_reading");
CONFIG_FLOAT(laser_max_reading, "sim.laser.max_reading");

}  // namespace sim

// std::random_device rd;
std::mt19937 gen(0);

std_msgs::Header MakeHeader(const std::string& frame_id) {
  static uint32_t seq = 0;
  std_msgs::Header header;
  header.seq = (++seq);
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  return header;
}

sensor_msgs::LaserScan MakeScan(const util::Pose& robot_pose,
                                const util::Map& map,
                                const float noise_stddev) {
  std::normal_distribution<> noise_dist(0.0f, noise_stddev);

  sensor_msgs::LaserScan scan;
  scan.header = MakeHeader("laser");
  scan.angle_min = sim::CONFIG_laser_min_angle;
  scan.angle_max = sim::CONFIG_laser_max_angle;
  scan.angle_increment = sim::CONFIG_laser_angle_delta;
  scan.range_min = sim::CONFIG_laser_min_reading;
  scan.range_max = sim::CONFIG_laser_max_reading;
  scan.scan_time = 0;
  scan.time_increment = 0;

  for (int ray_idx = 0; ray_idx < sim::CONFIG_laser_num_readings; ++ray_idx) {
    const float angle = math_util::AngleMod(sim::CONFIG_laser_min_angle +
                                            sim::CONFIG_laser_angle_delta *
                                                static_cast<float>(ray_idx) +
                                            robot_pose.rot);
    const util::Pose ray(robot_pose.tra, angle);
    const float dist =
        map.MinDistanceAlongRay(ray,
                                sim::CONFIG_laser_min_reading,
                                sim::CONFIG_laser_max_reading - kEpsilon);
    scan.ranges.push_back(dist + noise_dist(gen));
  }

  return scan;
}

util::Twist AddExecutionOdomNoise(util::Twist move) {
  std::normal_distribution<> along_arc_dist(
      0.0f, sim::CONFIG_kArcExecStdDev * move.tra.norm());
  std::normal_distribution<> rotation_dist(
      0.0f, sim::CONFIG_kRotateExecStdDev * move.rot + 0.005);
  move.tra.x() += along_arc_dist(gen);
  move.rot += rotation_dist(gen);
  return move;
}

util::Twist AddReadingOdomNoise(util::Twist move) {
  std::normal_distribution<> along_arc_dist(
      0.0f, sim::CONFIG_kArcReadStdDev * move.tra.norm());
  std::normal_distribution<> rotation_dist(
      0.0f, sim::CONFIG_kRotateReadStdDev * move.rot);
  move.tra.x() += along_arc_dist(gen);
  move.rot += rotation_dist(gen);
  return move;
}

util::Twist commanded_velocity;

void CommandedVelocityCallback(const geometry_msgs::Twist& nv) {
  commanded_velocity = util::Twist(nv);
}

int main(int argc, char** argv) {
  util::PrintCurrentWorkingDirectory();
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/nav_config.lua",
       "src/ServiceRobotControlStack/control_stack/config/sim_config.lua"});
  ros::init(argc, argv, "simulator");

  ros::NodeHandle n;

  ros::Publisher initial_pose_pub =
      n.advertise<geometry_msgs::Twist>("true_pose", 1);
  ros::Publisher scan_pub =
      n.advertise<sensor_msgs::LaserScan>(constants::kLaserTopic, 10);
  ros::Publisher odom_pub =
      n.advertise<nav_msgs::Odometry>(constants::kOdomTopic, 10);
  ros::Publisher map_pub = n.advertise<visualization_msgs::Marker>("map", 10);
  ros::Publisher initial_pose_vis_pub =
      n.advertise<visualization_msgs::MarkerArray>("true_pose_vis", 1);

  ros::Subscriber command_sub = n.subscribe(
      constants::kCommandVelocityTopic, 10, &CommandedVelocityCallback);

  static constexpr float kLoopRate = 10;

  ros::Rate loop_rate(kLoopRate);

  const util::Map map(sim::CONFIG_kMap);
  util::Pose current_pose(sim::CONFIG_kStartPositionX,
                          sim::CONFIG_kStartPositionY,
                          sim::CONFIG_kStartPositionTheta);

  while (ros::ok()) {
    const util::Twist executed_move =
        AddExecutionOdomNoise(commanded_velocity / kLoopRate);
    const util::Twist reported_move = AddReadingOdomNoise(executed_move);
    current_pose = geometry::FollowTrajectory(
        current_pose, executed_move.tra.x(), executed_move.rot);

    scan_pub.publish(MakeScan(current_pose, map, sim::CONFIG_kLaserStdDev));
    nav_msgs::Odometry odom_msg;
    odom_msg.header = MakeHeader("base_link");
    odom_msg.twist.twist = (reported_move * kLoopRate).ToTwist();
    odom_pub.publish(odom_msg);
    initial_pose_pub.publish(current_pose.ToTwist());
    visualization_msgs::MarkerArray arr;
    visualization::DrawPose(
        current_pose, "map", "true_pose_vis", 1, 1, 1, 1, &arr);
    initial_pose_vis_pub.publish(arr);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
