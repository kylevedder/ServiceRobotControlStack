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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <string>
#include <vector>

#include "cs/util/constants.h"
#include "cs/util/physics.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

namespace util {
namespace physics {

util::Twist ApplyCommandLimits(util::Twist commanded_velocity,
                               const float time_delta,
                               const util::Twist& current_velocity,
                               const float max_tra_velocity,
                               const float max_tra_acceleration,
                               const float max_rot_velocity,
                               const float max_rot_acceleration) {
  NP_CHECK(time_delta >= 0);
  if (time_delta < kEpsilon) {
    return current_velocity;
  }
  // Cap translational velocity.
  if (commanded_velocity.tra.squaredNorm() > math_util::Sq(max_tra_velocity)) {
    commanded_velocity.tra =
        commanded_velocity.tra.normalized() * max_tra_velocity;
  }
  // Cap rotational velocity.
  if (fabs(commanded_velocity.rot) > max_rot_velocity) {
    commanded_velocity.rot =
        math_util::Sign(commanded_velocity.rot) * max_rot_velocity;
  }
  // m/s - m/s -> m/s / s = m/s^2
  const util::Twist acceleration =
      (commanded_velocity - current_velocity) / time_delta;
  NP_FINITE_VEC(acceleration.tra);
  NP_FINITE(acceleration.rot);
  // Cap translational acceleration.
  if (acceleration.tra.squaredNorm() > math_util::Sq(max_tra_acceleration)) {
    // m/s = m/s + m/s^2 * s
    commanded_velocity.tra =
        current_velocity.tra +
        acceleration.tra.normalized() * max_tra_acceleration * time_delta;
  }
  // Cap rotational acceleration.
  if (fabs(acceleration.rot) > math_util::Sq(max_rot_acceleration)) {
    // rad/s = rad/s + rad/s^2 * s
    commanded_velocity.rot =
        current_velocity.rot +
        math_util::Sign(acceleration.rot) * max_rot_acceleration * time_delta;
  }
  return commanded_velocity;
}

CommandDeltaStraight ComputeCommandDeltaStraight(
    const util::Pose& current_position_wf,
    util::Twist commanded_velocity_rf,
    const float time_delta,
    const util::Twist& current_velocity_rf) {
  CommandDeltaStraight cd;
  cd.current_position_wf = current_position_wf;
  const float acceleration_times_time =
      (commanded_velocity_rf.tra.x() - current_velocity_rf.tra.x());
  // dx = vi * dt + 1/2 a * t^2 = vi * dt + 1/2 (a * t) * t
  const float dx = current_velocity_rf.tra.x() * time_delta +
                   0.5 * acceleration_times_time * time_delta;
  cd.end_position_wf = current_position_wf;
  cd.end_position_wf.tra += geometry::Heading(current_position_wf.rot) * dx;
  cd.end_velocity_rf = commanded_velocity_rf;
  return cd;
}

float RotationCircleRadius(const util::Twist& commanded_v) {
  NP_FINITE_VEC(commanded_v.tra);
  NP_FINITE(commanded_v.rot);
  const float linear_speed = commanded_v.tra.x();     // m/s
  const float rot_speed = std::abs(commanded_v.rot);  // rad / s
  NP_CHECK(rot_speed >= kEpsilon);
  const float radius = std::abs(linear_speed) / rot_speed;
  NP_FINITE(radius);
  NP_CHECK_VAL(radius >= 0, radius);
  return radius;
}

Eigen::Vector2f RotationCircleCenter(const util::Pose& pose,
                                     const util::Twist& velocity,
                                     const float& radius) {
  NP_FINITE(radius);
  NP_CHECK(pose.IsFinite());
  NP_CHECK(velocity.IsFinite());
  const int direction = math_util::Sign(velocity.rot);
  const Eigen::Vector2f forward = geometry::Heading(pose.rot);
  const Eigen::Vector2f towards_center =
      (Eigen::Rotation2Df(kPi * direction / 2) * forward);
  NP_FINITE_VEC(towards_center);
  return pose.tra + towards_center * radius;
}

util::Pose RotationCircleDistancePose(const util::Pose& robot_wf,
                                      const float rotation_velocity,
                                      const float circle_radius,
                                      const float dx,
                                      const float time_delta) {
  NP_FINITE(dx);

  // Handle turn in place which would cause NaNs in later math.
  if (circle_radius <= kEpsilon) {
    return robot_wf + util::Pose(0, 0, rotation_velocity * time_delta);
  }

  const int rotation_sign = math_util::Sign(rotation_velocity);

  // All of the following math is in robot frame.
  const float signed_radius = circle_radius * rotation_sign;

  const float angle_rf = dx / circle_radius;
  NP_FINITE(angle_rf);
  const Eigen::Vector2f final_pose_rf_tra(
      std::sin(angle_rf) * circle_radius,
      -std::cos(angle_rf) * signed_radius + signed_radius);
  NP_FINITE_VEC(final_pose_rf_tra);

  // Convert transform from robot frame to world frame.
  util::Pose result_wf = robot_wf;
  result_wf.tra += Eigen::Rotation2Df(robot_wf.rot) * final_pose_rf_tra;
  result_wf.rot = math_util::AngleMod(result_wf.rot + angle_rf * rotation_sign);
  NP_CHECK(result_wf.IsFinite());
  return result_wf;
}

util::Pose RotationCircleEndPose(const util::Pose& start_pose_wf,
                                 const util::Twist current_velocity_rf,
                                 const util::Twist commanded_velocity_rf,
                                 const float time_delta,
                                 const float circle_radius) {
  NP_CHECK(current_velocity_rf.IsFinite());
  NP_CHECK(commanded_velocity_rf.IsFinite());
  NP_CHECK_GE(time_delta, 0);

  const float acceleration_times_time =
      (commanded_velocity_rf.tra.x() - current_velocity_rf.tra.x());
  // dx = vi * dt + 1/2 a * t^2 = vi * dt + 1/2 (a * t) * t
  const float dx = commanded_velocity_rf.tra.x() * time_delta +
                   0.5 * acceleration_times_time * time_delta;

  return RotationCircleDistancePose(
      start_pose_wf, commanded_velocity_rf.rot, circle_radius, dx, time_delta);
}

CommandDeltaCurve ComputeCommandDeltaCurve(
    const util::Pose& current_position_wf,
    util::Twist commanded_velocity_rf,
    const float time_delta,
    const util::Twist& current_velocity_rf) {
  // We assume that the rotational velocity of the desired command is achieved
  // instantly, allowing us to treat this as a simpler problem of traveling
  // around a circle rather than having to solve for rotational velocity and
  // translational velocity simultaneously.
  CommandDeltaCurve cc;
  cc.current_position_wf = current_position_wf;
  cc.rotate_circle_radius = RotationCircleRadius(commanded_velocity_rf);
  cc.rotate_circle_center_wf = RotationCircleCenter(
      current_position_wf, commanded_velocity_rf, cc.rotate_circle_radius);
  cc.end_position_wf = RotationCircleEndPose(current_position_wf,
                                             current_velocity_rf,
                                             commanded_velocity_rf,
                                             time_delta,
                                             cc.rotate_circle_radius);
  cc.end_velocity_rf = commanded_velocity_rf;
  return cc;
}

CommandDelta ComputeCommandDelta(const util::Pose& current_position_wf,
                                 const util::Twist& current_velocity_rf,
                                 const util::Twist& commanded_velocity_rf,
                                 const float time_delta) {
  NP_CHECK(commanded_velocity_rf.IsFinite());
  if (std::abs(commanded_velocity_rf.rot) < kEpsilon) {
    // Straight case, no curve.
    return CommandDelta(ComputeCommandDeltaStraight(current_position_wf,
                                                    commanded_velocity_rf,
                                                    time_delta,
                                                    current_velocity_rf));
  }

  return CommandDelta(ComputeCommandDeltaCurve(current_position_wf,
                                               commanded_velocity_rf,
                                               time_delta,
                                               current_velocity_rf));
}

StopDelta ComputeFullStop(const CommandDelta& command_delta,
                          const float max_tra_acceleration) {
  NP_CHECK_GT(max_tra_acceleration, kEpsilon);
  // We ignore rotational deceleration and assume that translational
  // deceleration is the only concern, with the robot traveling along the circle
  // prescribed by its rotational velocity.

  // vf^2 = vi^2 + 2 a dx
  // (vf^2 - vi^2) / (2 a) =  dx
  // (0 - vi^2) / (2 a) =  dx

  const util::Twist& velocity_rf = command_delta.GetEndVelocity();
  const float dx = math_util::Sign(velocity_rf.tra.x()) *
                   math_util::Sq(velocity_rf.tra.x()) /
                   (2 * max_tra_acceleration);

  // 0m/s = vi m/s + (-a m/s^2 * t sec)
  // -vi m/s =  (-a m/s^2 * t sec)
  // -vi m/s / -a m/s^2 =  t sec
  const float time_delta = (velocity_rf.tra.x() / max_tra_acceleration);

  StopDelta sd;
  sd.current_position_wf = command_delta.GetEndPosition();
  if (command_delta.type == CommandDelta::Type::STRAIGHT) {
    // Straight line in the direction of the current pose.
    const Eigen::Vector2f delta =
        geometry::Heading(command_delta.straight.end_position_wf.rot) * dx;
    sd.stop_position_wf = command_delta.straight.end_position_wf;
    sd.stop_position_wf.tra += delta;
    return sd;
  }

  sd.stop_position_wf =
      RotationCircleDistancePose(sd.current_position_wf,
                                 velocity_rf.rot,
                                 command_delta.curve.rotate_circle_radius,
                                 dx,
                                 time_delta);
  return sd;
}

}  // namespace physics
}  // namespace util
