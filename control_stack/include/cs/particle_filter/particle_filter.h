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
  util::Map map_;

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
