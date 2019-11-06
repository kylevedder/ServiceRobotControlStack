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
#include <vector>

#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"

namespace cs {
namespace obstacle_avoidance {

class ObstacleDetector {
 public:
  ObstacleDetector() = delete;
  explicit ObstacleDetector(util::Map const& map);

  void UpdateObservation(const util::Pose& observation_pose,
                         const util::LaserScan& observation,
                         ros::Publisher* pub);

  void DrawDynamic(ros::Publisher* pub) const;

  const std::vector<util::Wall>& GetDynamicWalls() const;

 private:
  std::vector<Eigen::Vector2f> GetNonMapPoints(
      const util::Pose& observation_pose,
      const util::LaserScan& observation) const;

  util::Map map_;
  std::vector<util::Wall> dynamic_walls_;
};

}  // namespace obstacle_avoidance
}  // namespace cs
