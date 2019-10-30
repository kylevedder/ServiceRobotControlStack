#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "cs/util/geometry.h"

#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>

namespace util {

struct Wall {
  Eigen::Vector2f p1;
  Eigen::Vector2f p2;
  Wall() : p1(), p2(){};
  Wall(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) : p1(p1), p2(p2){};
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