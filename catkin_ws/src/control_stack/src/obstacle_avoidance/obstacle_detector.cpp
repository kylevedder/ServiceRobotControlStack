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
#include "cs/obstacle_avoidance/obstacle_detector.h"

#include <visualization_msgs/MarkerArray.h>

#include <limits>
#include <utility>
#include <vector>

#include "config_reader/macros.h"
#include "cs/motion_planning/trajectory_rollout.h"
#include "cs/util/laser_scan.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"

namespace cs {
namespace obstacle_avoidance {

namespace od_params {
CONFIG_FLOAT(kMinDistanceThreshold, "od.kMinDistanceThreshold");
CONFIG_FLOAT(kDistanceFromMax, "od.kDistanceFromMax");
CONFIG_FLOAT(kProposedTranslationStdDev, "od.kProposedTranslationStdDev");
CONFIG_FLOAT(kProposedRotationStdDev, "od.kProposedRotationStdDev");

CONFIG_FLOAT(max_dist_between_readings,
             "od.clustering.max_dist_between_readings");
CONFIG_FLOAT(min_distance_btw_readings_to_reason_angle,
             "od.clustering.min_distance_btw_readings_to_reason_angle");
CONFIG_FLOAT(line_similarity, "od.clustering.line_similarity");

CONFIG_STRING(map_tf_frame, "frames.map_tf_frame");

CONFIG_FLOAT(is_wall_threshold, "od.is_wall_threshold");
}  // namespace od_params

ObstacleDetector::ObstacleDetector(const util::Map& map)
    : map_(map), random_gen_(0) {}

bool IsMapObservation(const float& distance_to_wall) {
  return (distance_to_wall < od_params::CONFIG_is_wall_threshold);
}

std::vector<Eigen::Vector2f> ObstacleDetector::GetNonMapPoints(
    const util::Pose& observation_pose,
    const util::LaserScan& observation) const {
  const float min_depth = od_params::CONFIG_kMinDistanceThreshold;
  const float max_depth = observation.ros_laser_scan_.range_max -
                          od_params::CONFIG_kDistanceFromMax;
  const auto points = observation.TransformPointsFrameSparse(
      observation_pose.ToAffine(), [min_depth, max_depth](const float& f) {
        return f > min_depth && f < max_depth;
      });

  std::vector<Eigen::Vector2f> v;
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

  const auto in_same_cluster = [&start_v](
                                   const Eigen::Vector2f& current_cluster_line,
                                   const Eigen::Vector2f& prev,
                                   const Eigen::Vector2f& curr) -> bool {
    const Eigen::Vector2f delta = (curr - prev);
    if (delta.squaredNorm() >
        math_util::Sq(od_params::CONFIG_max_dist_between_readings)) {
      return false;
    }

    if (start_v == prev) {
      return true;
    }

    if (current_cluster_line.squaredNorm() <
            math_util::Sq(
                od_params::CONFIG_min_distance_btw_readings_to_reason_angle) ||
        delta.squaredNorm() <
            math_util::Sq(
                od_params::CONFIG_min_distance_btw_readings_to_reason_angle)) {
      return true;
    }

    const Eigen::Vector2f current_line_dir = current_cluster_line.normalized();
    const Eigen::Vector2f new_segment_dir = delta.normalized();
    const float dot = current_line_dir.dot(new_segment_dir);

    return fabs(dot) > od_params::CONFIG_line_similarity;
  };

  size_t prev_idx = cluster_start_idx;
  for (size_t i = cluster_start_idx + 1; i < points.size(); ++i) {
    NP_CHECK(prev_idx < points.size());
    NP_CHECK(i < points.size());
    const Eigen::Vector2f& prev_v = points[prev_idx];
    const Eigen::Vector2f current_cluster_line = prev_v - start_v;
    const Eigen::Vector2f& curr_v = points[i];
    if (!in_same_cluster(current_cluster_line, prev_v, curr_v)) {
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
                                         const util::LaserScan& observation) {
  UpdateObservation(observation_pose, observation, nullptr);
}

void ObstacleDetector::UpdateObservation(const util::Pose& observation_pose,
                                         const util::LaserScan& observation,
                                         ros::Publisher* pub) {
  static constexpr bool kDebug = false;
  dynamic_map_.walls.clear();
  const bool should_publish = (pub != nullptr);

  static visualization_msgs::Marker old_marker;

  const auto non_map_points = GetNonMapPoints(observation_pose, observation);
  for (const auto& p : non_map_points) {
    dynamic_map_.walls.push_back({p, p});
  }

  if (kDebug && should_publish) {
    ROS_INFO("Rendering %zu non-map points", non_map_points.size());
    visualization_msgs::Marker new_marker =
        visualization::PointsToLineList(non_map_points,
                                        observation_pose,
                                        od_params::CONFIG_map_tf_frame,
                                        "non_map_pts",
                                        0,
                                        0,
                                        1);
    old_marker.action = old_marker.DELETE;
    pub->publish(old_marker);
    pub->publish(new_marker);
    old_marker = new_marker;
  }
}

void ObstacleDetector::DrawDynamic(ros::Publisher* pub) const {
  // ROS_INFO("Obstacle detector found: %zu obstacles", dynamic_walls_.size());
  pub->publish(visualization::DrawWalls(
      dynamic_map_.walls, "est_map", "dynamic_walls_ns"));
}

const util::Map& ObstacleDetector::GetDynamicMap() const {
  return dynamic_map_;
}

}  // namespace obstacle_avoidance
}  // namespace cs