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
CONFIG_FLOAT(kDistanceFromMax, "od.kDistanceFromMax");

CONFIG_FLOAT(rotation_drive_threshold, "control.rotation_drive_threshold");
CONFIG_FLOAT(rotation_p, "control.rotation_p");
CONFIG_FLOAT(translation_p, "control.translation_p");
CONFIG_FLOAT(goal_deadzone_tra, "control.goal_deadzone_tra");
CONFIG_INT(laser_deadzone_left_min, "laser.laser_deadzone_left_min");
CONFIG_INT(laser_deadzone_left_max, "laser.laser_deadzone_left_max");
CONFIG_INT(laser_deadzone_right_min, "laser.laser_deadzone_right_min");
CONFIG_INT(laser_deadzone_right_max, "laser.laser_deadzone_right_max");
}  // namespace params

struct CallbackWrapper {
  util::Map empty_map_;
  cs::obstacle_avoidance::ObstacleDetector obstacle_detector_;

  CallbackWrapper() : empty_map_(), obstacle_detector_(empty_map_) {}

  util::Twist CommandVelocity(const util::Twist& desired_command,
                              const float& time_delta) {
    return obstacle_detector_.MakeCommandSafe(desired_command,
                                              time_delta,
                                              params::CONFIG_kCollisionRollout,
                                              params::CONFIG_kRobotRadius,
                                              params::CONFIG_kSafetyMargin);
  }

  std::tuple<util::LaserScan, std::array<util::Twist, 3>, util::Map>
  LaserCallback(const sensor_msgs::LaserScan& msg) {
    util::LaserScan laser(msg);
    laser.ClearDataInIndexRange(params::CONFIG_laser_deadzone_right_min,
                                params::CONFIG_laser_deadzone_right_max);
    laser.ClearDataInIndexRange(params::CONFIG_laser_deadzone_left_min,
                                params::CONFIG_laser_deadzone_left_max);
    laser.ClearDataFilter([&laser](const float& e) {
      return e > laser.ros_laser_scan_.range_min &&
             e < laser.ros_laser_scan_.range_max -
                     params::CONFIG_kDistanceFromMax;
    });

    obstacle_detector_.UpdateObservation({0, 0, 0}, laser, nullptr);
    const auto& dynamic_map = obstacle_detector_.GetDynamicMap();

    static constexpr float kDeltaTime = 0.1;

    const util::Twist forward_command =
        obstacle_detector_.ApplyCommandLimits({2, 0, 0}, kDeltaTime);
    const util::Twist left_command =
        obstacle_detector_.ApplyCommandLimits({2, 0, 0.03}, kDeltaTime);
    const util::Twist right_command =
        obstacle_detector_.ApplyCommandLimits({2, 0, -0.03}, kDeltaTime);

    const util::Twist forward_commanded_velocity =
        CommandVelocity(forward_command, kDeltaTime);
    const util::Twist left_commanded_velocity =
        CommandVelocity(left_command, kDeltaTime);
    const util::Twist right_commanded_velocity =
        CommandVelocity(right_command, kDeltaTime);

    //    std::cout << "Forward: " << forward_command << " vs "
    //              << forward_commanded_velocity << std::endl;
    //    std::cout << "Left: " << left_command << " vs " <<
    //    left_commanded_velocity
    //              << std::endl;
    //    std::cout << "Right: " << right_command << " vs "
    //              << right_commanded_velocity << std::endl;

    return {laser,
            {{forward_commanded_velocity,
              left_commanded_velocity,
              right_commanded_velocity}},
            dynamic_map};
  }
};

std::tuple<std::string, std::string, std::string, std::string, std::string>
GetArgs(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "Usage: [bag file] [laser file] [commands file] [dynamic map "
                 "file] [trajectory file]"
              << std::endl;
    exit(0);
  }
  return {argv[1], argv[2], argv[3], argv[4], argv[5]};
}

void WriteResults(std::ofstream* laser_file,
                  std::ofstream* commands_file,
                  std::ofstream* dynamic_map_file,
                  std::ofstream* trajectory_file,
                  const util::LaserScan& laser_scan,
                  const std::array<util::Twist, 3>& commands,
                  const util::Map& dynamic_map) {
  static constexpr float kNoDataRange = 50;
  {
    auto points = laser_scan.TransformPointsFrame(Eigen::Affine2f::Identity(),
                                                  kNoDataRange);
    std::stringstream ss;
    for (const auto& p : points) {
      ss << p.x() << " " << p.y() << " ";
    }
    ss << "\n";
    (*laser_file) << ss.str();
  }
  {
    std::stringstream ss;
    for (const auto& cmd : commands) {
      ss << cmd.tra.x() << " " << cmd.rot << " ";
    }
    ss << "\n";
    (*commands_file) << ss.str();
  }
  {
    std::stringstream ss;
    ss << "[";
    for (const auto& wall : dynamic_map.walls) {
      ss << "(" << wall.p1.x() << ", " << wall.p1.y() << ", " << wall.p2.x()
         << ", " << wall.p2.y() << "), ";
    }
    ss << "]\n";
    (*dynamic_map_file) << ss.str();
  }
  {
    std::stringstream ss;
    ss << "[";
    for (const auto& cmd : commands) {
      cs::obstacle_avoidance::TrajectoryRollout tr(
          {0, 0, 0}, {0, 0, 0}, cmd, params::CONFIG_kCollisionRollout);
      ss << "(" << tr.final_pose.tra.x() << ", " << tr.final_pose.tra.y()
         << ", " << tr.final_pose.rot << ", " << tr.rotate_circle_center.x()
         << ", " << tr.rotate_circle_center.y() << ", "
         << tr.rotate_circle_radius << ", " << tr.achieved_vel_pose.tra.x()
         << ", " << tr.achieved_vel_pose.tra.y() << ", "
         << tr.achieved_vel_pose.rot << "), ";
    }
    ss << "]\n";
    (*trajectory_file) << ss.str();
  }
}

int main(int argc, char** argv) {
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/bag_nav_config.lua"});
  auto res = GetArgs(argc, argv);
  rosbag::Bag bag(std::get<0>(res));
  CallbackWrapper cw;
  std::ofstream laser_file(std::get<1>(res));
  std::ofstream commands_file(std::get<2>(res));
  std::ofstream dynamic_map_file(std::get<3>(res));
  std::ofstream trajectory_file(std::get<4>(res));

  int iter = 0;
  for (const auto& m : rosbag::View(bag)) {
    if (m.getTopic() == "/scan") {
      auto s = m.instantiate<sensor_msgs::LaserScan>();
      CHECK(s != nullptr);
      auto commands = cw.LaserCallback(*s);
      WriteResults(&laser_file,
                   &commands_file,
                   &dynamic_map_file,
                   &trajectory_file,
                   std::get<0>(commands),
                   std::get<1>(commands),
                   std::get<2>(commands));
      ++iter;
    }
  }
  bag.close();
  laser_file.close();
  commands_file.close();

  return 0;
}
