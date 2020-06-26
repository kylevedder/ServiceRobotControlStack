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

#include "cs/localization/particle_filter.h"
#include "cs/state_estimation/state_estimator.h"
#include "cs/util/datastructures/circular_buffer.h"
#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"

namespace cs {
namespace state_estimation {

namespace params {
CONFIG_FLOAT(command_bias, "od.command_bias");
}  // namespace params

class PFStateEstimator : public StateEstimator {
 private:
  cs::localization::ParticleFilter pf_;
  util::Twist last_command_;
  util::Twist last_odom_velocity_;
  util::Pose estimated_pose_;

  static constexpr size_t kTimeBufferSize = 5;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize> odom_times_;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize> laser_times_;

  float GetTimeDelta(
      const cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize>& b)
      const {
    if (b.size() <= 1) {
      return kEpsilon;
    }
    const double total_time_delta = (b.back() - b.front()).toSec();
    const double iterations = static_cast<double>(b.size() - 1);
    return static_cast<float>(total_time_delta / iterations);
  }

 public:
  PFStateEstimator() = delete;
  PFStateEstimator(const util::vector_map::VectorMap& map,
                   const util::Pose& initial_pose)
      : StateEstimator(), pf_(map, initial_pose) {}
  ~PFStateEstimator() = default;

  void UpdateLaser(const util::LaserScan& laser, const ros::Time& time) {
    pf_.UpdateObservation(laser);
    NP_CHECK(laser.ros_laser_scan_.header.stamp == time);
    laser_times_.push_back(time);
    estimated_pose_ = pf_.WeightedCentroid();
  }

  void UpdateOdom(const util::Twist& odom_velocity, const ros::Time& time) {
    last_odom_velocity_ = odom_velocity;
    // Estimates the current velocity and scales it down to change since odom
    // update using smoothed odom time delta.
    const auto delta = GetEstimatedVelocity() * GetOdomTimeDelta();
    pf_.UpdateOdom(delta.tra.x(), delta.rot);
    odom_times_.push_back(time);
    estimated_pose_ = pf_.WeightedCentroid();
  }

  void UpdateLastCommand(const util::Twist& cmd) { last_command_ = cmd; }

  util::Pose GetEstimatedPose() const { return estimated_pose_; }

  util::Twist GetEstimatedVelocity() const {
    return last_command_ * params::CONFIG_command_bias +
           last_odom_velocity_ * (1.0f - params::CONFIG_command_bias);
  }

  float GetOdomTimeDelta() const { return GetTimeDelta(odom_times_); }

  float GetLaserTimeDelta() const { return GetTimeDelta(laser_times_); }

  void Visualize(ros::Publisher* pub) const { pf_.DrawParticles(pub); }
};

}  // namespace state_estimation
}  // namespace cs
