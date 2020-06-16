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
#include "cs/motion_planning/trajectory_rollout.h"

#include <algorithm>

#include "config_reader/macros.h"
#include "cs/util/constants.h"
#include "cs/util/map.h"
#include "cs/util/physics.h"
#include "shared/math/geometry.h"

namespace cs {
namespace motion_planning {

namespace tr_params {
CONFIG_FLOAT(kMaxTraAcc, "limits.kMaxTraAcc");
CONFIG_FLOAT(kMaxTraVel, "limits.kMaxTraVel");
CONFIG_FLOAT(kMaxRotAcc, "limits.kMaxRotAcc");
CONFIG_FLOAT(kMaxRotVel, "limits.kMaxRotVel");
CONFIG_FLOAT(decelerate_scaler, "safety.decelerate_scaler");

CONFIG_FLOAT(min_trajectory_rotation, "od.min_trajectory_rotation");
}  // namespace tr_params

float AchievedVelocityTime(const util::Twist& current_v,
                           const util::Twist& commanded_v) {
  const float vel_delta = (commanded_v.tra.x() - current_v.tra.x());  // m/s
  return vel_delta / tr_params::CONFIG_kMaxTraAcc;
}

Eigen::Vector2f CircleCenter(const util::Pose& pose,
                             const util::Twist& velocity,
                             const float& radius) {
  NP_FINITE(radius);
  const int direction = math_util::Sign(velocity.rot);
  NP_FINITE(direction);
  const Eigen::Vector2f forward = geometry::Heading(pose.rot);
  NP_FINITE_VEC2(forward);
  const Eigen::Vector2f towards_center =
      (Eigen::Rotation2Df(kPi * direction / 2) * forward);
  NP_FINITE_VEC2(towards_center);
  return pose.tra + towards_center * radius;
}

util::Pose AchievedVelocityPose(const util::Pose& start_pose,
                                const util::Twist& current_v,
                                const float& time) {
  const float delta_x =
      current_v.tra.x() * time +
      0.5f * tr_params::CONFIG_kMaxTraAcc * math_util::Sq(time);
  const float& rot = start_pose.rot;
  const Eigen::Vector2f position_delta(std::cos(rot) * delta_x,
                                       std::sin(rot) * delta_x);
  return {start_pose.tra + position_delta, start_pose.rot};
}

float RotationCircleRadius(const util::Twist& commanded_v) {
  NP_FINITE(commanded_v.tra.x());
  NP_FINITE(commanded_v.rot);
  const float linear_speed = commanded_v.tra.x();  // m/s
  const float rot_speed = fabs(commanded_v.rot);   // rad / s
  NP_CHECK(fabs(commanded_v.rot) >= kEpsilon);
  const float radius = fabs(linear_speed) / rot_speed;
  NP_FINITE(radius);
  NP_CHECK_VAL(radius >= 0, radius);
  return radius;
}

util::Pose RotateFinalPose(const util::Pose& start_pose,
                           const float& rotate_radians,
                           const float& radius) {
  const Eigen::Vector2f robot_frame_delta(
      std::sin(fabs(rotate_radians)) * radius,
      (1.0f - std::cos(fabs(rotate_radians))) * radius *
          math_util::Sign(rotate_radians));
  const Eigen::Vector2f start_frame_delta =
      Eigen::Rotation2Df(start_pose.rot) * robot_frame_delta;
  return start_pose + util::Pose(start_frame_delta, rotate_radians);
}

bool IsCollidingLinear(const util::Pose& pose1,
                       const util::Pose& pose2,
                       const util::Wall& wall,
                       const float& min_dist_threshold) {
  const auto& p1 = pose1.tra;
  const auto& p2 = pose2.tra;
  const auto& w1 = wall.p1;
  const auto& w2 = wall.p2;

  const float dist = geometry::MinDistanceLineLine(p1, p2, w1, w2);
  NP_FINITE(dist);
  return dist <= min_dist_threshold;
}

TrajectoryRollout::TrajectoryRollout(const util::Pose& start_pose,
                                     const util::Twist& current_v,
                                     util::Twist commanded_v,
                                     const float rollout_duration)
    : start_pose(start_pose),
      current_v(current_v),
      commanded_v(commanded_v),
      rollout_duration(rollout_duration),
      achieved_vel_pose({0, 0}, 0),
      rotate_circle_center(0, 0),
      rotate_circle_radius(0),
      final_pose({0, 0}, 0) {
  NP_CHECK(current_v.tra.y() == 0);
  NP_CHECK(commanded_v.tra.y() == 0);
  NP_FINITE(current_v.tra.x());
  NP_FINITE(commanded_v.tra.x());
  NP_FINITE(current_v.rot);
  NP_FINITE(commanded_v.rot);

  commanded_v = util::physics::ApplyCommandLimits(commanded_v,
                                                  rollout_duration,
                                                  current_v,
                                                  tr_params::CONFIG_kMaxTraVel,
                                                  tr_params::CONFIG_kMaxTraAcc,
                                                  tr_params::CONFIG_kMaxRotVel,
                                                  tr_params::CONFIG_kMaxRotAcc);

  const auto cd = util::physics::ComputeCommandDelta(
      start_pose, current_v, commanded_v, rollout_duration);

  const auto cs = util::physics::ComputeFullStop(
      cd, tr_params::CONFIG_kMaxTraAcc * tr_params::CONFIG_decelerate_scaler);

  this->achieved_vel_pose = cd.GetEndPosition();
  if (cd.type == util::physics::CommandDelta::Type::CURVE) {
    this->rotate_circle_center = cd.curve.rotate_circle_center_wf;
    this->rotate_circle_radius = cd.curve.rotate_circle_radius;
  }
  this->final_pose = cs.stop_position_wf;
}

bool TrajectoryRollout::IsColliding(const util::Wall& wall,
                                    const float radius) const {
  if (IsCollidingLinear(start_pose, achieved_vel_pose, wall, radius)) {
    return true;
  }

  if (IsCollidingLinear(achieved_vel_pose, final_pose, wall, radius)) {
    return true;
  }

  return false;
}
}  // namespace motion_planning
}  // namespace cs
