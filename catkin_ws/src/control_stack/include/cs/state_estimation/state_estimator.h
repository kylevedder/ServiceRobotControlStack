#pragma once
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
#include <ros/ros.h>
#include <array>
#include <random>

#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"

namespace cs {
namespace state_estimation {

class StateEstimator {
 public:
  StateEstimator() = default;
  virtual ~StateEstimator() = default;

  virtual void UpdateLaser(const util::LaserScan& laser,
                           const ros::Time& time) = 0;

  virtual void UpdateOdom(const util::Twist& odom_velocity,
                          const ros::Time& time) = 0;

  virtual void UpdateLastCommand(const util::Twist& cmd) = 0;

  virtual util::Pose GetEstimatedPose() const = 0;

  virtual util::Twist GetEstimatedVelocity() const = 0;

  virtual void Visualize(ros::Publisher* pub) const = 0;

  virtual float GetOdomTimeDelta() const = 0;

  virtual float GetLaserTimeDelta() const = 0;
};

}  // namespace state_estimation
}  // namespace cs
