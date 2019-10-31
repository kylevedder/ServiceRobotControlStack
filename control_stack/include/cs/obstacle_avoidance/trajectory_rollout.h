#pragma once

#include "cs/util/map.h"
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
  float rotate_circle_achieved_vel_angle;
  util::Pose final_pose;
  float rotate_circle_finale_pose_angle;

  TrajectoryRollout() = delete;
  TrajectoryRollout(const util::Pose& start_pose, const util::Pose& current_v,
                    const util::Pose& commanded_v,
                    const float rollout_duration);

  bool IsColliding(const util::Wall& wall, const float& robot_radius) const;
};

}  // namespace obstacle_avoidance
}  // namespace cs