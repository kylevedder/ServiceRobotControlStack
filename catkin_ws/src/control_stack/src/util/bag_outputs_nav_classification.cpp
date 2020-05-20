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

std::tuple<std::string, std::string> GetArgs(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: [outputs file] [trajectory file]" << std::endl;
    exit(0);
  }
  return {argv[1], argv[2]};
}

void WriteResults(std::ofstream* trajectory_file,
                  const std::array<util::Twist, 3>& commands) {
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

util::Twist GetTwist(const int index) {
  const std::array<util::Twist, 4> lst = {{util::Twist(0, 0, 0),
                                           util::Twist(0.02, 0, 0.2),
                                           util::Twist(0.02, 0, 0),
                                           util::Twist(0.02, 0, -0.2)}};
  CHECK(index < static_cast<int>(lst.size()));
  return lst[index];
}

std::array<util::Twist, 3> GetCommandsFromFile(std::string line) {
  std::istringstream iss(line);
  //    util::Twist center(0, 0, 0);
  //    float center_sin = 0;
  //    float center_cos = 0;
  //    util::Twist left(0, 0, 0);
  //    float left_sin = 0;
  //    float left_cos = 0;
  //    util::Twist right(0, 0, 0);
  //    float right_sin = 0;
  //    float right_cos = 0;
  //    iss >> center.tra.x() >> left.tra.x() >> right.tra.x() >> center_cos >>
  //        left_cos >> right_cos >> center_sin >> left_sin >> right_sin;
  //    center.rot = std::atan2(center_sin, center_cos);
  //    left.rot = std::atan2(left_sin, left_cos);
  //    right.rot = std::atan2(right_sin, right_cos);
  //
  //  return {{center, left, right}};

  int class_id = 0;
  iss >> class_id;

  CHECK_PRINT_VAL(class_id >= 0 && class_id < 64, class_id);

  const int center_id = class_id / 16;
  const int left_id = (class_id % 16) / 4;
  const int right_id = (class_id % 4);

  return {{GetTwist(center_id), GetTwist(left_id), GetTwist(right_id)}};
}

int main(int argc, char** argv) {
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/bag_nav_config.lua"});
  auto res = GetArgs(argc, argv);
  std::ifstream commands_file(std::get<0>(res));
  std::ofstream trajectory_file(std::get<1>(res));

  std::string line;
  while (std::getline(commands_file, line)) {
    WriteResults(&trajectory_file, GetCommandsFromFile(line));
  }
  return 0;
}
