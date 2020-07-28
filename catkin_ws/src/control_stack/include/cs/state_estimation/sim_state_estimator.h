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

class SimStateEstimator : public StateEstimator {
 private:
  util::Pose ground_truth_pose_;
  util::Twist last_odom_velocity_;
  ros::Subscriber ground_truth_pose_sub_;

  static constexpr size_t kTimeBufferSize = 5;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize> odom_times_;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize> laser_times_;

  float GetTimeDelta(
      const cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize>& b)
      const {
    if (b.size() <= 1) {
      return 2 * kEpsilon;
    }
    const double total_time_delta = (b.back() - b.front()).toSec();
    const double iterations = static_cast<double>(b.size() - 1);
    return static_cast<float>(total_time_delta / iterations);
  }

  void GroundTruthLocationCallback(const geometry_msgs::PoseStamped& pose) {
    ground_truth_pose_ = util::Pose(pose.pose);
  }

 public:
  SimStateEstimator() = delete;
  explicit SimStateEstimator(ros::NodeHandle* n)
      : ground_truth_pose_(), last_odom_velocity_() {
    ground_truth_pose_sub_ =
        n->subscribe("/simulator_true_pose",
                     10,
                     &SimStateEstimator::GroundTruthLocationCallback,
                     this);
  }
  ~SimStateEstimator() = default;

  void UpdateLaser(const util::LaserScan& laser, const ros::Time& time) {
    NP_CHECK(laser.ros_laser_scan_.header.stamp == time);
    laser_times_.push_back(time);
  }

  void UpdateOdom(const util::Twist&, const ros::Time& time) {
    odom_times_.push_back(time);
  }

  void UpdateLastCommand(const util::Twist& cmd) { last_odom_velocity_ = cmd; }

  util::Pose GetEstimatedPose() const { return ground_truth_pose_; }

  util::Twist GetEstimatedVelocity() const { return last_odom_velocity_; }

  float GetOdomTimeDelta() const { return GetTimeDelta(odom_times_); }

  float GetLaserTimeDelta() const { return GetTimeDelta(laser_times_); }

  void Visualize(ros::Publisher*) const {}
};

}  // namespace state_estimation
}  // namespace cs
