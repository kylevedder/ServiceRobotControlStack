#include "cs/util/map.h"

#include <fstream>
#include <sstream>

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

float Map::MinDistanceAlongRay(const util::Pose& ray, const float min_depth,
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
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "map_ns";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  for (const Wall& w : walls) {
    geometry_msgs::Point p1;
    p1.x = w.p1.x();
    p1.y = w.p1.y();
    geometry_msgs::Point p2;
    p2.x = w.p2.x();
    p2.y = w.p2.y();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  marker.scale.x = 0.02;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  return marker;
}

}  // namespace util
