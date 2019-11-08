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
#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include "cs/util/geometry.h"
#include "cs/util/map.h"
#include "cs/util/visualization.h"

namespace util {

Map::Map(const std::string& filepath) {
  std::fstream map_file(filepath);
  if (!map_file.is_open()) {
    std::cerr << "Failed to open " << filepath << std::endl;
  }
  CHECK(map_file.is_open());

  std::string line;
  while (std::getline(map_file, line)) {
    float x1 = 0;
    float y1 = 0;
    float x2 = 0;
    float y2 = 0;
    std::istringstream iss(line);
    if (!(iss >> x1 >> y1 >> x2 >> y2)) {
      break;
    }
    walls.push_back({{x1, y1}, {x2, y2}});
  }
}

float Map::MinDistanceAlongRay(const util::Pose& ray,
                               const float min_depth,
                               const float max_depth) const {
  NP_FINITE(ray.tra.x());
  NP_FINITE(ray.tra.x());
  NP_FINITE(max_depth);

  Eigen::Vector2f delta =
      Eigen::Rotation2Df(ray.rot) * Eigen::Vector2f(max_depth, 0);
  for (const util::Wall& w : this->walls) {
    const Eigen::Vector2f& ray_start = ray.tra;
    const Eigen::Vector2f& ray_end = ray.tra + delta;
    const auto res =
        geometry::CheckLineLineIntersection(w.p1, w.p2, ray_start, ray_end);
    if (!res.first) {
      continue;
    }
    const Eigen::Vector2f& collision_end = res.second;
    const Eigen::Vector2f& collision_delta = (collision_end - ray.tra);
    if (delta.squaredNorm() > collision_delta.squaredNorm()) {
      delta = collision_delta;
    }
  }
  if (delta.squaredNorm() < math_util::Sq(kMinReading)) {
    return min_depth;
  }
  return delta.norm();
}

visualization_msgs::Marker Map::ToMarker() const {
  return visualization::DrawWalls(walls, "map", "map_ns");
}

float Map::MinDistanceToWall(const Eigen::Vector2f& observation) const {
  float min_distance = std::numeric_limits<float>::max();
  for (const Wall& w : walls) {
    const Eigen::Vector2f wall_dir = (w.p1 - w.p2).normalized();
    const Eigen::Vector2f p1_to_obs = observation - w.p1;
    const Eigen::Vector2f p2_to_obs = observation - w.p2;
    const float dist_p1_sq =
        (p1_to_obs - ((p1_to_obs).dot(wall_dir) * wall_dir)).squaredNorm();
    const float dist_p2_sq =
        (p2_to_obs - ((p2_to_obs).dot(wall_dir) * wall_dir)).squaredNorm();
    min_distance = std::min({min_distance, dist_p1_sq, dist_p2_sq});
  }
  return std::sqrt(min_distance);
}

}  // namespace util
