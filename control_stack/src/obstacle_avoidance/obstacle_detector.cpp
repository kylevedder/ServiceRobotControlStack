
#include "cs/obstacle_avoidance/obstacle_detector.h"

#include <cs/util/visualization.h>
#include <visualization_msgs/MarkerArray.h>
#include "cs/util/laser_scan.h"

namespace cs {
namespace obstacle_avoidance {

ObstacleDetector::ObstacleDetector(util::Map const& map) : map_(map) {}

bool IsMapObservation(const float& distance_to_wall) {
  static constexpr float kWallThreshold = 0.1;
  return (distance_to_wall < kWallThreshold);
}

std::vector<Eigen::Vector2f> ObstacleDetector::GetNonMapPoints(
    const util::Pose& observation_pose,
    const util::LaserScan& observation) const {
  std::vector<Eigen::Vector2f> v;
  const auto points =
      observation.TransformPointsFrameSparse(observation_pose.ToAffine());
  for (const auto& p : points) {
    if (!IsMapObservation(map_.MinDistanceToWall(p))) {
      v.push_back(p);
    }
  }
  return v;
}

size_t GetClusterEndIdx(const std::vector<Eigen::Vector2f>& points,
                        const size_t cluster_start_idx) {
  NP_CHECK(!points.empty());
  NP_CHECK(points.size() > cluster_start_idx);

  const Eigen::Vector2f& start_v = points[cluster_start_idx];

  static const auto in_same_cluster = [&start_v](
                                          const Eigen::Vector2f& prev,
                                          const Eigen::Vector2f& curr) -> bool {
    static constexpr float kMaxDistanceBetweenReadings = 0.3;
    static constexpr float kMinDistanceBetweenReadingsToReasonAngle = 0.05;

    const Eigen::Vector2f delta = (curr - prev);
    if (delta.squaredNorm() > math_util::Sq(kMaxDistanceBetweenReadings)) {
      return false;
    }

    if (start_v == prev ||
        delta.squaredNorm() <
            math_util::Sq(kMinDistanceBetweenReadingsToReasonAngle)) {
      return true;
    }

    const Eigen::Vector2f current_line_dir = (prev - start_v).normalized();
    const Eigen::Vector2f new_segment_dir = delta.normalized();
    const float dot = current_line_dir.dot(new_segment_dir);

    static constexpr float kSimilarity = 0.17;  // Cos(80 deg)
    return fabs(dot) > kSimilarity;
  };

  size_t prev_idx = cluster_start_idx;
  for (size_t i = cluster_start_idx + 1; i < points.size(); ++i) {
    const Eigen::Vector2f& prev_v = points[prev_idx];
    const Eigen::Vector2f& curr_v = points[i];
    if (!in_same_cluster(prev_v, curr_v)) {
      return prev_idx;
    }
    prev_idx = i;
  }

  return (points.size() - 1);
}

util::Wall FitWallToCluster(const std::vector<Eigen::Vector2f>& points,
                            const size_t cluster_start_idx,
                            const size_t cluster_end_idx) {
  NP_CHECK(cluster_start_idx < points.size());
  NP_CHECK(cluster_end_idx < points.size());
  NP_CHECK(cluster_start_idx <= cluster_end_idx);

  const auto& v1 = points[cluster_start_idx];
  const auto& v2 = points[cluster_end_idx];
  return {v1, v2};
}

void ObstacleDetector::UpdateObservation(const util::Pose& observation_pose,
                                         const util::LaserScan& observation,
                                         ros::Publisher* pub) {
  dynamic_walls_.clear();

  static visualization_msgs::MarkerArray old_markers;
  for (visualization_msgs::Marker& marker : old_markers.markers) {
    marker.action = marker.DELETE;
  }

  const auto non_map_points = GetNonMapPoints(observation_pose, observation);

  if (non_map_points.empty()) {
    pub->publish(old_markers);
    ROS_INFO("Published %zu non-map points", non_map_points.size());
    return;
  }

  visualization_msgs::MarkerArray new_markers;

  size_t num_clusters = 0;
  size_t cluster_start = 0;
  while (cluster_start < non_map_points.size()) {
    const size_t cluster_end = GetClusterEndIdx(non_map_points, cluster_start);
    dynamic_walls_.push_back(
        FitWallToCluster(non_map_points, cluster_start, cluster_end));

    const auto color = visualization::IndexToDistinctRBG(num_clusters);
    const auto& r = std::get<0>(color);
    const auto& g = std::get<1>(color);
    const auto& b = std::get<2>(color);
    std::vector<Eigen::Vector2f> cluster_points;
    for (size_t i = cluster_start; i <= cluster_end; ++i) {
      cluster_points.push_back(non_map_points[i]);
    }

    ROS_INFO("Cluster size: %zu", (cluster_end - cluster_start + 1));

    visualization::PointsToSpheres(cluster_points, "map", "non_points_ns",
                                   &new_markers, r, g, b);
    new_markers.markers.push_back(visualization::ToLine(
        dynamic_walls_.back().p1, dynamic_walls_.back().p2, "map",
        "colored_wall_ns", num_clusters, r, g, b));

    cluster_start = cluster_end + 1;
    ++num_clusters;
  }

  pub->publish(old_markers);
  pub->publish(new_markers);
  old_markers = new_markers;
  ROS_INFO("Published %zu non-map points", non_map_points.size());
  ROS_INFO("Found %zu clusters", num_clusters);
}

void ObstacleDetector::DrawDynamic(ros::Publisher* pub) const {
  ROS_INFO("Obstacle detector found: %zu obstacles", dynamic_walls_.size());
  pub->publish(
      visualization::DrawWalls(dynamic_walls_, "map", "dynamic_walls_ns"));
}

const std::vector<util::Wall>& ObstacleDetector::GetDynamicWalls() const {
  return dynamic_walls_;
}

}  // namespace obstacle_avoidance
}  // namespace cs
