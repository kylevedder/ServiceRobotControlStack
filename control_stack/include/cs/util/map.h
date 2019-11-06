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

#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>
#include "cs/util/pose.h"

namespace util {

struct Wall {
  Eigen::Vector2f p1;
  Eigen::Vector2f p2;
  Wall() : p1(), p2() {}
  Wall(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) : p1(p1), p2(p2) {}
};

struct Map {
  std::vector<Wall> walls;

  Map() = default;
  explicit Map(const std::string& filepath);

  float MinDistanceAlongRay(const util::Pose& ray, const float min_depth,
                            const float max_depth) const;

  float MinDistanceToWall(const Eigen::Vector2f& observation) const;

  visualization_msgs::Marker ToMarker() const;
};

}  // namespace util
