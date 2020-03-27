#pragma once
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
#include <ros/ros.h>
#include <random>
#include <vector>

#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"

namespace cs {
namespace obstacle_avoidance {

class ObstacleDetector {
 public:
  ObstacleDetector() = delete;
  explicit ObstacleDetector(const util::Map& map);

  void UpdateObservation(const util::Pose& observation_pose,
                         const util::LaserScan& observation);
  void UpdateObservation(const util::Pose& observation_pose,
                         const util::LaserScan& observation,
                         ros::Publisher* pub);

  void UpdateOdom(const util::Pose& pose, const util::Twist& velocity);

  void UpdateCommand(const util::Twist& prior_commanded_velocity);

  void DrawDynamic(ros::Publisher* pub) const;

  const util::Map& GetDynamicMap() const;

  bool IsCommandColliding(const util::Twist& commanded_velocity,
                          const float rollout_duration,
                          const float robot_radius,
                          const float safety_margin) const;

  util::Twist MakeCommandSafe(util::Twist commanded_velocity,
                              const float time_delta,
                              const float rollout_duration,
                              const float robot_radius,
                              const float safety_margin);

  util::Twist ApplyCommandLimits(util::Twist c, const float& time_delta) const;

  util::Twist EstimateCurrentVelocity() const;

 private:
  std::vector<Eigen::Vector2f> GetNonMapPoints(
      const util::Pose& observation_pose,
      const util::LaserScan& observation) const;

  bool StartedInCollision(const float robot_radius) const;

  const util::Map& map_;
  util::Map dynamic_map_;
  util::Pose estimated_pose_;
  util::Twist odom_velocity_;
  util::Twist prior_commanded_velocity_;
  std::mt19937 random_gen_;
};

}  // namespace obstacle_avoidance
}  // namespace cs
