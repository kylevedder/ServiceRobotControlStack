#pragma once
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

  bool IsColliding(const util::Wall& wall,
                   const float& robot_radius,
                   const float safety_margin) const;
};

}  // namespace obstacle_avoidance
}  // namespace cs
