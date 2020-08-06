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

#include "cs/controllers/escape_collision_controller.h"

#include <utility>
#include <vector>

namespace cs {
namespace controllers {

namespace params {

CONFIG_FLOAT(robot_radius, "pf.kRobotRadius");
CONFIG_FLOAT(safety_margin, "pf.kSafetyMargin");
CONFIG_FLOAT(local_inflation, "path_finding.local_robot_inflation");
CONFIG_FLOAT(num_safety_margins, "esc_collision.num_safety_margins");

CONFIG_STRING(map_tf_frame, "frames.map_tf_frame");
CONFIG_STRING(base_link_tf_frame, "frames.base_tf_frame");
CONFIG_STRING(laser_tf_frame, "frames.laser_tf_frame");

}  // namespace params

EscapeCollisionController::EscapeCollisionController(
    cs::main::DebugPubWrapper* dpw,
    const util::LaserScan& laser,
    const util::vector_map::VectorMap& map,
    const state_estimation::StateEstimator& state_estimator,
    const obstacle_avoidance::ObstacleDetector& obstacle_detector,
    const motion_planning::PIDController& motion_planner)
    : Controller(
          dpw, laser, map, state_estimator, obstacle_detector, motion_planner),
      escape_waypoint_() {}

void DrawWaypoint(cs::main::DebugPubWrapper* dpw, const util::Pose& goal) {
  visualization_msgs::MarkerArray goal_marker;
  visualization::DrawPose(
      goal, params::CONFIG_map_tf_frame, "goal_pose", 0, 1, 0, 1, &goal_marker);
  dpw->goal_pub_.publish(goal_marker);
}

EscapeCollisionWaypoint ComputeEscapeWaypoint(
    const Eigen::Vector2f& agent_point_wf,
    const std::vector<Eigen::Vector2f>& laser_points_wf) {
  const float total_margin =
      (params::CONFIG_robot_radius + params::CONFIG_safety_margin) *
      params::CONFIG_local_inflation;
  if (laser_points_wf.empty()) {
    return {false, agent_point_wf, agent_point_wf};
  }

  Eigen::Vector2f closest_point_wf = laser_points_wf.front();
  float closest_point_dist = (closest_point_wf - agent_point_wf).squaredNorm();
  for (const auto& p : laser_points_wf) {
    const float d = (agent_point_wf - p).squaredNorm();
    if (d < closest_point_dist) {
      closest_point_wf = p;
      closest_point_dist = d;
    }
  }

  if (closest_point_dist > math_util::Sq(total_margin)) {
    return {false, agent_point_wf, agent_point_wf};
  }

  const Eigen::Vector2f waypoint_wf =
      -(closest_point_wf - agent_point_wf).normalized() * total_margin *
          params::CONFIG_num_safety_margins +
      agent_point_wf;
  return {true, waypoint_wf, closest_point_wf};
}

std::pair<ControllerType, util::Twist> EscapeCollisionController::Execute() {
  static constexpr bool kDebug = false;
  const auto est_pose = state_estimator_.GetEstimatedPose();
  auto laser_points_wf = laser_.TransformPointsFrameSparse(
      est_pose.ToAffine(), [this](const float& d) {
        return (d > laser_.ros_laser_scan_.range_min) &&
               (d <= laser_.ros_laser_scan_.range_max);
      });

  // Ensure that the prior colliding point is not forgotten to prevent
  // flip-flopping.
  if (escape_waypoint_.initialized) {
    laser_points_wf.push_back(escape_waypoint_.colliding_point);
  }

  auto current_escape_waypoint =
      ComputeEscapeWaypoint(est_pose.tra, laser_points_wf);
  if (!current_escape_waypoint.initialized) {
    if (!escape_waypoint_.initialized) {
      if (kDebug) {
        ROS_INFO("No escape waypoint found, transitioning to nav");
      }
      return {ControllerType::NAVIGATION, {}};
    }
    current_escape_waypoint = escape_waypoint_;
  }

  colliding_marker_ =
      visualization::PointToSphere(current_escape_waypoint.colliding_point,
                                   params::CONFIG_map_tf_frame,
                                   "colliding_point",
                                   0,
                                   0,
                                   1);
  dpw_->colliding_point_pub_.publish(colliding_marker_);
  waypoint_marker_ =
      visualization::PointToSphere(current_escape_waypoint.waypoint,
                                   params::CONFIG_map_tf_frame,
                                   "waypoint",
                                   0,
                                   0,
                                   0);
  dpw_->colliding_point_pub_.publish(waypoint_marker_);

  if (motion_planner_.AtPoint(current_escape_waypoint.waypoint)) {
    if (kDebug) {
      ROS_INFO("At escape waypoint, transitioning to nav");
    }
    return {ControllerType::NAVIGATION, {}};
  }

  const util::Pose desired_pose(current_escape_waypoint.waypoint, est_pose.rot);
  DrawWaypoint(dpw_, desired_pose);
  const util::Twist command = motion_planner_.EscapeCollision(desired_pose);
  escape_waypoint_ = current_escape_waypoint;

  return {ControllerType::ESCAPE_COLLISION, command};
}

void EscapeCollisionController::Reset() {
  escape_waypoint_ = {};
  colliding_marker_.action = colliding_marker_.DELETE;
  waypoint_marker_.action = waypoint_marker_.DELETE;
  dpw_->colliding_point_pub_.publish(colliding_marker_);
  dpw_->colliding_point_pub_.publish(waypoint_marker_);
}

}  // namespace controllers
}  // namespace cs
