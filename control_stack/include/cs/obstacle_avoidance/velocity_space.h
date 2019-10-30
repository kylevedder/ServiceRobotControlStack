#pragma once

#include "cs/util/pose.h"

namespace cs {
namespace obstacle_avoidance {

struct TrajectoryRollout {
  util::Pose start_pose;
  util::Pose current_v;
  util::Pose commanded_v;
  float rollout_duration;
  util::Pose achieved_vel_pose;
  Eigen::Vector2f rotate_circle_center;
  float rotate_circle_radius;
  util::Pose final_pose;

  TrajectoryRollout() = delete;
  TrajectoryRollout(const util::Pose& start_pose, const util::Pose& current_v,
                    const util::Pose& commanded_v, const float rollout_duration)
      : start_pose(start_pose),
        current_v(current_v),
        commanded_v(commanded_v),
        rollout_duration(rollout_duration),
        achieved_vel_pose(),
        rotate_circle_center(0, 0),
        rotate_circle_radius(0),
        final_pose() {
    NP_CHECK(current_v.tra.y() == 0);
    NP_CHECK(commanded_v.tra.y() == 0);

    static constexpr float kMinRotatonRadSec = 0.05f;

    const float achieved_vel_time = AchievedVelocityTime();
    NP_FINITE(achieved_vel_time);
    achieved_vel_pose = AchievedVelocityPose(achieved_vel_time);
    NP_CHECK(achieved_vel_pose.IsFinite());
    const float rotate_time = rollout_duration - achieved_vel_time;
    NP_FINITE(rotate_time);

    if (fabs(commanded_v.rot) < kMinRotatonRadSec) {
      // Handle small rotation case where numerically unstable.
      const float delta_dist = commanded_v.tra.x() * rotate_time;
      rotate_circle_center = achieved_vel_pose.tra;
      rotate_circle_radius = 0;
      final_pose =
          achieved_vel_pose +
          util::Pose({math_util::Cos(achieved_vel_pose.rot) * delta_dist,
                      math_util::Sin(achieved_vel_pose.rot) * delta_dist},
                     0);
      NP_CHECK(final_pose.IsFinite());

    } else {
      // Handle general case which is numerically stable.
      const float rotate_radians = commanded_v.rot * rotate_time;
      rotate_circle_radius = RotationCircleRadius();
      NP_FINITE(rotate_circle_radius);
      rotate_circle_center =
          CircleCenter(achieved_vel_pose, commanded_v, rotate_circle_radius);
      NP_FINITE(rotate_circle_center.x());
      NP_FINITE(rotate_circle_center.y());
      final_pose = RotateFinalPose(achieved_vel_pose, rotate_time,
                                   rotate_radians, rotate_circle_radius);
      NP_CHECK(final_pose.IsFinite());
    }
  }

 private:
  float AchievedVelocityTime() {
    const float vel_delta = (commanded_v.tra.x() - current_v.tra.x());  // m/s
    return vel_delta / kRobotMaxAccel;
  }

  Eigen::Vector2f CircleCenter(const util::Pose& pose,
                               const util::Pose& velocity,
                               const float& radius) {
    const int direction = math_util::Sign(velocity.rot);
    const Eigen::Vector2f forward(math_util::Cos(pose.rot),
                                  math_util::Sin(pose.rot));
    const Eigen::Vector2f towards_center =
        (Eigen::Rotation2Df(kPi / 2) * forward) * direction;
    return pose.tra + towards_center * radius;
  }

  util::Pose AchievedVelocityPose(const float& time) {
    const float delta_x =
        current_v.tra.x() * time + 0.5f * kRobotMaxAccel * math_util::Sq(time);
    const float& rot = start_pose.rot;
    const Eigen::Vector2f position_delta(math_util::Cos(rot) * delta_x,
                                         math_util::Sin(rot) * delta_x);
    return {start_pose.tra + position_delta, start_pose.rot};
  }

  float RotationCircleRadius() {
    const float linear_speed = commanded_v.tra.x();  // m/s
    const float rot_speed = fabs(commanded_v.rot);   // rad / s
    return linear_speed / rot_speed;
  }

  util::Pose RotateFinalPose(const util::Pose& start_pose, const float& time,
                             const float& rotate_radians, const float& radius) {
    const Eigen::Vector2f robot_frame_delta(
        math_util::Sin(fabs(rotate_radians)) * radius,
        (1.0f - math_util::Cos(fabs(rotate_radians))) * radius *
            math_util::Sign(rotate_radians));
    const Eigen::Vector2f start_frame_delta =
        Eigen::Rotation2Df(start_pose.rot) * robot_frame_delta;
    return start_pose + util::Pose(start_frame_delta, rotate_radians);
  }
};

}  // namespace obstacle_avoidance
}  // namespace cs