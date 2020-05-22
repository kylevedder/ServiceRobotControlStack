// Copyright 2019 kvedder@seas.upenn.edu
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

#include "cs/path_finding/astar.h"

#include <cs/motion_planning/pid.h>
#include <shared/util/array_util.h>
#include <algorithm>
#include <limits>
#include <string>

#include "config_reader/macros.h"

namespace cs {
namespace motion_planning {

namespace params {
CONFIG_FLOAT(rotation_drive_threshold, "control.rotation_drive_threshold");
CONFIG_FLOAT(rotation_p, "control.rotation_p");
CONFIG_FLOAT(translation_p, "control.translation_p");
CONFIG_FLOAT(goal_deadzone_tra, "control.goal_deadzone_tra");

CONFIG_FLOAT(kMaxTraAcc, "limits.kMaxTraAcc");
CONFIG_FLOAT(kMaxTraVel, "limits.kMaxTraVel");
CONFIG_FLOAT(kMaxRotAcc, "limits.kMaxRotAcc");
CONFIG_FLOAT(kMaxRotVel, "limits.kMaxRotVel");

CONFIG_FLOAT(robot_radius, "pf.kRobotRadius");
CONFIG_FLOAT(safety_margin, "pf.kSafetyMargin");
CONFIG_FLOAT(collision_rollout, "pf.kCollisionRollout");

CONFIG_FLOAT(translational_cost_scale_factor, "od.kTranslationCostScaleFactor");

}  // namespace params

util::Twist PIDController::DriveToPoint(const util::Map& dynamic_map,
                                        const Eigen::Vector2f& waypoint) {
  complete_map_ = map_.Merge(dynamic_map);
  const auto estimated_velocity = state_estimator_.GetEstimatedVelocity();

  const auto limited_command = ApplyCommandLimits(ProposeCommand(waypoint));

  if (!IsCommandColliding(limited_command)) {
    return limited_command;
  }

  auto costs = array_util::MakeArray<5>(0.0f);
  std::array<util::Twist, 5> alternate_commands = {
      {{0, 0, 0},
       {0, 0, params::CONFIG_kMaxRotVel},
       {0, 0, -params::CONFIG_kMaxRotVel},
       {estimated_velocity.tra, params::CONFIG_kMaxRotVel},
       {estimated_velocity.tra, -params::CONFIG_kMaxRotVel}}};

  NP_CHECK(alternate_commands[0] == util::Twist(0, 0, 0));

  for (auto& cmd : alternate_commands) {
    cmd = ApplyCommandLimits(cmd);
  }

  for (size_t i = 0; i < alternate_commands.size(); ++i) {
    costs[i] = AlternateCommandCost(limited_command, alternate_commands[i]);
  }

  size_t best_idx = array_util::ArgMin(costs);
  // If every command causes a crash, we command full brakes, limited by
  // acceleration constraints.
  if (costs[best_idx] >= std::numeric_limits<float>::max()) {
    best_idx = 0;
  }
  return alternate_commands[best_idx];
}

float PIDController::AlternateCommandCost(const util::Twist& desired,
                                          const util::Twist& alternate) const {
  const auto delta_pose = desired - alternate;
  if (IsCommandColliding(alternate)) {
    return std::numeric_limits<float>::max();
  }
  return math_util::Sq(delta_pose.tra.lpNorm<1>() *
                       params::CONFIG_translational_cost_scale_factor) +
         fabs(delta_pose.rot);
}

util::Twist PIDController::ProposeCommand(
    const Eigen::Vector2f& waypoint) const {
  const auto robot_heading = geometry::Heading(est_world_pose_.rot);
  const Eigen::Vector2f waypoint_delta = (waypoint - est_world_pose_.tra);
  const float distance_to_goal_sq = waypoint_delta.squaredNorm();
  NP_FINITE(distance_to_goal_sq);
  if (distance_to_goal_sq < Sq(params::CONFIG_goal_deadzone_tra)) {
    return {0, 0, 0};
  }

  const auto waypoint_heading = geometry::GetNormalizedOrZero(waypoint_delta);
  NP_FINITE_2F(waypoint_heading);
  const int angle_direction =
      math_util::Sign(geometry::Cross(robot_heading, waypoint_heading));
  NP_FINITE(angle_direction);
  const float angle = std::acos(robot_heading.dot(waypoint_heading));
  NP_FINITE(angle);
  NP_CHECK_VAL(angle >= 0 && angle <= kPi + kEpsilon, angle);
  float x = 0;
  if (angle < params::CONFIG_rotation_drive_threshold) {
    x = std::sqrt(distance_to_goal_sq);
    x *= math_util::Sign(waypoint_heading.dot(robot_heading));
  }
  return {x * params::CONFIG_translation_p,
          0,
          angle * angle_direction * params::CONFIG_rotation_p};
}

util::Twist PIDController::ApplyCommandLimits(util::Twist c) const {
  const float& time_delta = state_estimator_.GetLaserTimeDelta();
  static constexpr bool kDebug = false;
  NP_CHECK(time_delta >= 0);
  if (kDebug) {
    ROS_INFO("Time delta: %f", time_delta);
  }

  const util::Twist estimated_velocity =
      state_estimator_.GetEstimatedVelocity();

  if (time_delta < kEpsilon) {
    return estimated_velocity;
  }

  // Cap translational velocity.
  if (c.tra.squaredNorm() > math_util::Sq(params::CONFIG_kMaxTraVel)) {
    c.tra = c.tra.normalized() * params::CONFIG_kMaxTraVel;
  }
  // Cap rotational velocity.
  if (fabs(c.rot) > params::CONFIG_kMaxRotVel) {
    c.rot = math_util::Sign(c.rot) * params::CONFIG_kMaxRotVel;
  }

  // m/s - m/s -> m/s / s = m/s^2
  const util::Twist acceleration = (c - estimated_velocity) / time_delta;

  // Cap translational acceleration.
  if (acceleration.tra.squaredNorm() >
      math_util::Sq(params::CONFIG_kMaxTraAcc)) {
    // m/s = m/s + m/s^2 * s
    c.tra = estimated_velocity.tra + acceleration.tra.normalized() *
                                         params::CONFIG_kMaxTraAcc * time_delta;
  }

  // Cap rotational acceleration.
  if (fabs(acceleration.rot) > math_util::Sq(params::CONFIG_kMaxRotAcc)) {
    // rad/s = rad/s + rad/s^2 * s
    c.rot = estimated_velocity.rot + math_util::Sign(acceleration.rot) *
                                         params::CONFIG_kMaxRotAcc * time_delta;
  }

  return c;
}

bool PIDController::IsCommandColliding(
    const util::Twist& commanded_velocity) const {
  const float& rollout_duration = params::CONFIG_collision_rollout;
  const float& robot_radius = params::CONFIG_robot_radius;
  const float& safety_margin = params::CONFIG_safety_margin;

  static constexpr bool kDebug = false;
  const TrajectoryRollout tr(state_estimator_.GetEstimatedPose(),
                             state_estimator_.GetEstimatedVelocity(),
                             commanded_velocity,
                             rollout_duration);
  for (const auto& w : complete_map_.walls) {
    if (tr.IsColliding(w, robot_radius, safety_margin)) {
      if (kDebug) {
        ROS_INFO("Current command: (%f, %f), %f",
                 commanded_velocity.tra.x(),
                 commanded_velocity.tra.y(),
                 commanded_velocity.rot);
        ROS_INFO("End pose: (%f, %f), %f",
                 tr.final_pose.tra.x(),
                 tr.final_pose.tra.y(),
                 tr.final_pose.rot);
        ROS_INFO("Colliding with observed wall: (%f, %f) <-> (%f, %f)",
                 w.p1.x(),
                 w.p1.y(),
                 w.p2.x(),
                 w.p2.y());
      }
      return true;
    }
  }
  return false;
}

}  // namespace motion_planning
}  // namespace cs
