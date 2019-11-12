#pragma once
// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================
#include "cs/util/map.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"

namespace cs {
namespace obstacle_avoidance {

struct TrajectoryRollout {
  util::Pose start_pose;
  util::Twist current_v;
  util::Twist commanded_v;
  float rollout_duration;
  util::Pose achieved_vel_pose;
  Eigen::Vector2f rotate_circle_center;
  float rotate_circle_radius;
  float rotate_circle_achieved_vel_angle;
  util::Pose final_pose;
  float rotate_circle_finale_pose_angle;

  TrajectoryRollout() = delete;
  TrajectoryRollout(const util::Pose& start_pose,
                    const util::Twist& current_v,
                    const util::Twist& commanded_v,
                    const float rollout_duration);

  bool IsColliding(const util::Wall& wall, const float& robot_radius) const;
};

}  // namespace obstacle_avoidance
}  // namespace cs
