#include "cs/obstacle_avoidance/trajectory_rollout.h"

#include "cs/util/geometry.h"
#include "cs/util/map.h"

namespace cs {
namespace obstacle_avoidance {

float AchievedVelocityTime(const util::Pose& current_v,
                           const util::Pose& commanded_v) {
  const float vel_delta = (commanded_v.tra.x() - current_v.tra.x());  // m/s
  return vel_delta / kRobotMaxAccel;
}

Eigen::Vector2f CircleCenter(const util::Pose& pose, const util::Pose& velocity,
                             const float& radius) {
  NP_FINITE(radius);
  std::cout << "CircleCenter: " << velocity.tra.x() << ", " << velocity.rot
            << std::endl;
  const int direction = math_util::Sign(velocity.rot);
  NP_FINITE(direction);
  const Eigen::Vector2f forward = geometry::Heading(pose.rot);
  NP_FINITE_2F(forward);
  std::cout << "Forward: " << forward.x() << ", " << forward.y() << std::endl;
  const Eigen::Vector2f towards_center =
      (Eigen::Rotation2Df(kPi * direction / 2) * forward);
  NP_FINITE_2F(towards_center);
  std::cout << "Towards center " << towards_center.x() << ", "
            << towards_center.y() << std::endl;
  return pose.tra + towards_center * radius;
}

util::Pose AchievedVelocityPose(const util::Pose& start_pose,
                                const util::Pose& current_v,
                                const float& time) {
  const float delta_x =
      current_v.tra.x() * time + 0.5f * kRobotMaxAccel * math_util::Sq(time);
  const float& rot = start_pose.rot;
  const Eigen::Vector2f position_delta(math_util::Cos(rot) * delta_x,
                                       math_util::Sin(rot) * delta_x);
  return {start_pose.tra + position_delta, start_pose.rot};
}

float RotationCircleRadius(const util::Pose& commanded_v) {
  NP_FINITE(commanded_v.tra.x());
  NP_FINITE(commanded_v.rot);
  const float linear_speed = commanded_v.tra.x();  // m/s
  const float rot_speed = fabs(commanded_v.rot);   // rad / s
  NP_CHECK(fabs(commanded_v.rot) >= kEpsilon);
  const float radius = linear_speed / rot_speed;
  NP_FINITE(radius);
  return radius;
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

bool IsCollidingLinear(const util::Pose& pose1, const util::Pose& pose2,
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
                                     const util::Pose& current_v,
                                     const util::Pose& commanded_v,
                                     const float rollout_duration)
    : start_pose(start_pose),
      current_v(current_v),
      commanded_v(commanded_v),
      rollout_duration(rollout_duration),
      achieved_vel_pose({0, 0}, 0),
      rotate_circle_center(0, 0),
      rotate_circle_radius(0),
      rotate_circle_achieved_vel_angle(0),
      final_pose({0, 0}, 0),
      rotate_circle_finale_pose_angle(0) {
  NP_CHECK(current_v.tra.y() == 0);
  NP_CHECK(commanded_v.tra.y() == 0);
  NP_FINITE(current_v.tra.x());
  NP_FINITE(commanded_v.tra.x());
  NP_FINITE(current_v.rot);
  NP_FINITE(commanded_v.rot);

  static constexpr float kMinRotatonRadSec = 0.05f;

  const float achieved_vel_time = AchievedVelocityTime(current_v, commanded_v);
  NP_FINITE(achieved_vel_time);
  achieved_vel_pose =
      AchievedVelocityPose(start_pose, current_v, achieved_vel_time);
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
    NP_FINITE(commanded_v.rot);
    NP_FINITE(rotate_time);
    rotate_circle_radius = RotationCircleRadius(commanded_v);
    NP_FINITE(rotate_circle_radius);
    rotate_circle_center =
        CircleCenter(achieved_vel_pose, commanded_v, rotate_circle_radius);
    rotate_circle_achieved_vel_angle = math_util::AngleMod(
        geometry::Angle<float>(achieved_vel_pose.tra - rotate_circle_center));
    NP_FINITE(rotate_circle_achieved_vel_angle);
    const float rotate_delta = commanded_v.rot * rotate_time;
    rotate_circle_finale_pose_angle =
        math_util::AngleMod(rotate_circle_achieved_vel_angle + rotate_delta);
    NP_FINITE(rotate_circle_finale_pose_angle);

    NP_FINITE(rotate_circle_center.x());
    NP_FINITE(rotate_circle_center.y());
    final_pose = RotateFinalPose(achieved_vel_pose, rotate_time, rotate_delta,
                                 rotate_circle_radius);
    NP_CHECK(final_pose.IsFinite());
  }
}

bool TrajectoryRollout::IsColliding(const util::Wall& wall,
                                    const float& robot_radius) const {
  const float min_dist_threshold = robot_radius + kEpsilon;
  if (IsCollidingLinear(start_pose, achieved_vel_pose, wall,
                        min_dist_threshold)) {
    return true;
  }

  // Turn in place or no rotation.
  if (fabs(rotate_circle_radius) < kEpsilon) {
    return IsCollidingLinear(achieved_vel_pose, final_pose, wall,
                             min_dist_threshold);
  }

  const float dist = geometry::MinDistanceLineArc(
      wall.p1, wall.p2, rotate_circle_center, rotate_circle_radius,
      rotate_circle_achieved_vel_angle, rotate_circle_finale_pose_angle);
  NP_FINITE(dist);
  NP_CHECK_VAL(dist >= 0.0f, dist);
  return dist <= min_dist_threshold;
}
}  // namespace obstacle_avoidance
}  // namespace cs
