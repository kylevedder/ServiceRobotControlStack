#pragma once
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
#include "cs/motion_planning/command_scaler.h"
#include "cs/motion_planning/identity_command_scaler.h"
#include "cs/motion_planning/pid.h"
#include "cs/motion_planning/turtlebot_command_scaler.h"
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/path_finding/astar.h"
#include "cs/path_finding/global_path_finder.h"
#include "cs/state_estimation/pf_state_estimator.h"
#include "cs/state_estimation/sim_state_estimator.h"
#include "cs/state_estimation/state_estimator.h"
#include "cs/util/physics.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

namespace cs {
namespace main {

static constexpr bool kDebug = true;

struct StateMachineState {
  enum class State { EXITOBSTACLE, NAVIGATE };
  State state;
  util::Pose exit_obstacle_waypoint;
  StateMachineState() : state(State::NAVIGATE), exit_obstacle_waypoint() {}
};

struct CallbackWrapper {
  util::vector_map::VectorMap map_;
  std::unique_ptr<state_estimation::StateEstimator> state_estimator_;
  obstacle_avoidance::ObstacleDetector obstacle_detector_;
  motion_planning::PIDController motion_planner_;
  std::unique_ptr<motion_planning::CommandScaler> command_scaler_;
  path_finding::GlobalPathFinder<path_finding::AStar<3, 20000, false>>
      global_path_finder_;
  path_finding::AStar<5, 500, false> local_path_finder_;
  tf::TransformBroadcaster br_;
  util::Pose current_goal_;
  int current_goal_index_;

  StateMachineState state_;

  // Functionality pub/sub
  ros::Publisher position_pub_;
  ros::Publisher command_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber teleop_sub_;

  // Debug pub/sub
  ros::Publisher modified_laser_pub_;
  ros::Publisher particle_pub_;
  ros::Publisher map_pub_;
  ros::Publisher detected_walls_pub_;
  ros::Publisher robot_size_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher robot_path_pub_;

  CallbackWrapper() = delete;

  state_estimation::StateEstimator* MakeStateEstimator(ros::NodeHandle* n);

  motion_planning::CommandScaler* MakeCommandScaler();

  CallbackWrapper(const std::string& map_file, ros::NodeHandle* n);

  void GoalCallback(const geometry_msgs::Pose2D& msg);

  void CleanLaserScan(util::LaserScan* laser);

  void DrawPath(const path_finding::Path2f& p, const std::string& ns);

  void DrawGoal(const util::Pose& goal);

  void DrawRobot(const util::vector_map::VectorMap& full_map,
                 util::Twist command);

  util::Pose GetNextPose(const util::Pose& current_pose,
                         const path_finding::Path2f& path);

  void LaserCallback(const sensor_msgs::LaserScan& msg);

  void OdomCallback(const nav_msgs::Odometry& msg);

  void PublishTransforms();
};

}  // namespace main
}  // namespace cs
