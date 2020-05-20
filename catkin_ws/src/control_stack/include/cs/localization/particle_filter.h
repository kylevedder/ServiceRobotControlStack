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
#include <ros/ros.h>
#include <array>
#include <random>

#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"

namespace cs {
namespace localization {

using Time = float;

class MotionModel {
 private:
  std::random_device rd_;
  std::mt19937 gen_;

 public:
  MotionModel();

  util::Pose ForwardPredict(const util::Pose& pose_global_frame,
                            const float translation_robot_frame,
                            const float rotation_robot_frame);
};

class SensorModel {
 private:
  const util::Map& map_;

 public:
  explicit SensorModel(const util::Map& map);

  float GetProbability(const util::Pose& pose_global_frame,
                       const util::LaserScan& laser_scan,
                       util::LaserScan* filtered_laser_scan) const;
};

struct Particle {
  util::Pose pose;
  util::Pose prev_pose;
  util::LaserScan prev_filtered_laser;
  float weight;

  Particle() : pose(), weight(0.0f) {}

  Particle(const util::Pose& pose, const float weight)
      : pose(pose), prev_pose(pose), weight(weight) {}

  bool operator==(const Particle& other) const {
    return (pose == other.pose) && (weight == other.weight);
  }
};

class ParticleFilter {
 private:
  bool initialized_;
  std::random_device rd_;
  std::mt19937 gen_;
  static constexpr int kNumParticles = 50;
  using ParticleArray = std::array<Particle, kNumParticles>;
  ParticleArray particles_;
  MotionModel motion_model_;
  SensorModel sensor_model_;

  util::LaserScan ReweightParticles(const util::LaserScan& laser_scan);

  void ResampleParticles();

 public:
  ParticleFilter() = delete;

  explicit ParticleFilter(const util::Map& map);

  ParticleFilter(const util::Map& map, const util::Pose& start_pose);

  bool IsInitialized() const;

  void InitalizePose(const util::Pose& start_pose);

  void UpdateOdom(const float& translation, const float& rotation);

  void UpdateObservation(const util::LaserScan& laser_scan,
                         ros::Publisher* sampled_scan_pub);

  void UpdateObservation(const util::LaserScan& laser_scan);

  float ScoreObservation(const util::Pose& pose,
                         const util::LaserScan& laser_scan) const;

  void DrawParticles(ros::Publisher* particle_pub) const;

  util::Pose MaxWeight() const;

  util::Pose WeightedCentroid() const;
};
}  // namespace localization
}  // namespace cs
