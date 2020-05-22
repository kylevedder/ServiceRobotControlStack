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
#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include "cs/util/map.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"

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

Map::Map(std::vector<Wall> walls) : walls(std::move(walls)) {}

float Map::MinDistanceAlongRay(const util::Pose& ray,
                               const float& min_depth,
                               const float& max_depth) const {
  NP_FINITE(ray.tra.x());
  NP_FINITE(ray.tra.y());
  NP_FINITE(min_depth);
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
  if (delta.squaredNorm() < math_util::Sq(min_depth)) {
    return min_depth;
  }
  return delta.norm();
}

visualization_msgs::Marker Map::ToMarker() const {
  return visualization::DrawWalls(walls, "map", "map_ns");
}

float Map::MinDistanceToWall(const Eigen::Vector2f& observation) const {
  if (walls.empty()) {
    return std::numeric_limits<float>::max();
  }
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

Map Map::Merge(const Map& other) const {
  auto joined_walls = walls;
  joined_walls.insert(
      joined_walls.end(), other.walls.begin(), other.walls.end());
  return Map(joined_walls);
}

}  // namespace util
