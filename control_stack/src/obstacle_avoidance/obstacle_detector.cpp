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
#include <visualization_msgs/MarkerArray.h>
#include <limits>
#include <utility>
#include <vector>

#include "config_reader/macros.h"
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/obstacle_avoidance/trajectory_rollout.h"
#include "cs/util/laser_scan.h"
#include "cs/util/math_util.h"
#include "cs/util/visualization.h"

namespace cs {
namespace obstacle_avoidance {

namespace params {
CONFIG_FLOAT(kProposedTranslationStdDev, "od.kProposedTranslationStdDev");
CONFIG_FLOAT(kProposedRotationStdDev, "od.kProposedRotationStdDev");
CONFIG_FLOAT(kMinDistanceThreshold, "od.kMinDistanceThreshold");
CONFIG_FLOAT(kMaxTraVel, "limits.kMaxTraVel");
CONFIG_FLOAT(kMaxRotVel, "limits.kMaxRotVel");
}  // namespace params

ObstacleDetector::ObstacleDetector(util::Map const& map)
    : map_(map),
      current_pose_(0, 0, 0),
      current_velocity_(0, 0, 0),
      random_gen_(0) {}

bool IsMapObservation(const float& distance_to_wall) {
  static constexpr float kWallThreshold = 0.1;
  return (distance_to_wall < kWallThreshold);
}

std::vector<Eigen::Vector2f> ObstacleDetector::GetNonMapPoints(
    const util::Pose& observation_pose,
    const util::LaserScan& observation) const {
  std::vector<Eigen::Vector2f> v;
  const auto points = observation.TransformPointsFrameSparse(
      observation_pose.ToAffine(),
      [](const float& f) { return f > params::kMinDistanceThreshold; });
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

void ObstacleDetector::UpdateOdom(const util::Pose& pose,
                                  const util::Pose& velocity) {
  current_pose_ = pose;
  current_velocity_ = velocity;
}

void ObstacleDetector::UpdateObservation(const util::Pose& observation_pose,
                                         const util::LaserScan& observation) {
  UpdateObservation(observation_pose, observation, nullptr);
}

void ObstacleDetector::UpdateObservation(const util::Pose& observation_pose,
                                         const util::LaserScan& observation,
                                         ros::Publisher* pub) {
  static constexpr bool kDebug = false;
  dynamic_walls_.clear();

  static visualization_msgs::MarkerArray old_markers;
  for (visualization_msgs::Marker& marker : old_markers.markers) {
    marker.action = marker.DELETE;
  }

  const auto non_map_points = GetNonMapPoints(observation_pose, observation);

  if (non_map_points.empty()) {
    if (pub != nullptr) {
      pub->publish(old_markers);
      if (kDebug) {
        ROS_INFO("Published %zu non-map points", non_map_points.size());
      }
    }
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

    if (kDebug) {
      ROS_INFO("Cluster size: %zu", (cluster_end - cluster_start + 1));
    }

    visualization::PointsToSpheres(
        cluster_points, "map", "non_points_ns", &new_markers, r, g, b);
    new_markers.markers.push_back(
        visualization::ToLine(dynamic_walls_.back().p1,
                              dynamic_walls_.back().p2,
                              "map",
                              "colored_wall_ns",
                              num_clusters,
                              r,
                              g,
                              b));

    cluster_start = cluster_end + 1;
    ++num_clusters;
  }
  if (pub != nullptr) {
    pub->publish(old_markers);
    pub->publish(new_markers);
  }
  old_markers = new_markers;
  if (kDebug) {
    ROS_INFO("Published %zu non-map points", non_map_points.size());
    ROS_INFO("Found %zu clusters", num_clusters);
  }
  current_pose_ = observation_pose;
}

void ObstacleDetector::DrawDynamic(ros::Publisher* pub) const {
  // ROS_INFO("Obstacle detector found: %zu obstacles", dynamic_walls_.size());
  pub->publish(
      visualization::DrawWalls(dynamic_walls_, "map", "dynamic_walls_ns"));
}

const std::vector<util::Wall>& ObstacleDetector::GetDynamicWalls() const {
  return dynamic_walls_;
}

bool ObstacleDetector::IsCommandColliding(const util::Pose& commanded_velocity,
                                          const float rollout_duration,
                                          const float robot_radius) const {
  static constexpr bool kDebug = false;
  const TrajectoryRollout tr(
      current_pose_, current_velocity_, commanded_velocity, rollout_duration);
  for (const auto& w : dynamic_walls_) {
    if (tr.IsColliding(w, robot_radius)) {
      if (kDebug) {
        ROS_INFO("Current command: (%f, %f), %f",
                 commanded_velocity.tra.x(),
                 commanded_velocity.tra.y(),
                 commanded_velocity.rot);
        ROS_INFO("End pose: (%f, %f), %f",
                 tr.final_pose.tra.x(),
                 tr.final_pose.tra.y(),
                 tr.final_pose.rot);
        ROS_INFO("Colliding with observed wall: (%f, %f) <-> (%f, %f)",
                 w.p1.x(),
                 w.p1.y(),
                 w.p2.x(),
                 w.p2.y());
      }
      return true;
    }
  }
  for (const auto& w : map_.walls) {
    if (tr.IsColliding(w, robot_radius)) {
      if (kDebug) {
        ROS_INFO("Current command: (%f, %f), %f",
                 commanded_velocity.tra.x(),
                 commanded_velocity.tra.y(),
                 commanded_velocity.rot);
        ROS_INFO("End pose: (%f, %f), %f",
                 tr.final_pose.tra.x(),
                 tr.final_pose.tra.y(),
                 tr.final_pose.rot);
        ROS_INFO("Colliding with map wall: (%f, %f) <-> (%f, %f)",
                 w.p1.x(),
                 w.p1.y(),
                 w.p2.x(),
                 w.p2.y());
      }
      return true;
    }
  }
  return false;
}

util::Pose ObstacleDetector::ApplyCommandLimits(util::Pose p) const {
  if (p.tra.squaredNorm() < math_util::Sq(params::kMaxTraVel)) {
    p.tra = p.tra.normalized() * params::kMaxTraVel;
  }
  if (fabs(p.rot) > params::kMaxRotVel) {
    p.rot = math_util::Sign(p.rot) * params::kMaxRotVel;
  }
  return p;
}

util::Pose ObstacleDetector::MakeCommandSafe(util::Pose commanded_velocity,
                                             const float rollout_duration,
                                             const float robot_radius) {
  commanded_velocity = ApplyCommandLimits(commanded_velocity);
  static constexpr bool kDebug = false;
  if (kDebug) {
    ROS_INFO("Current position: (%f, %f), %f",
             current_pose_.tra.x(),
             current_pose_.tra.y(),
             current_pose_.rot);
    ROS_INFO("Current velocity: (%f, %f), %f",
             current_velocity_.tra.x(),
             current_velocity_.tra.y(),
             current_velocity_.rot);
    ROS_INFO("Current command: (%f, %f), %f",
             commanded_velocity.tra.x(),
             commanded_velocity.tra.y(),
             commanded_velocity.rot);
  }
  if (!IsCommandColliding(commanded_velocity, rollout_duration, robot_radius)) {
    return commanded_velocity;
  }

  static const util::Pose zero_command(0, 0, 0);
  // NP_CHECK(!IsCommandColliding(zero_command, rollout_duration,
  // robot_radius));

  static constexpr int kIterations = 50;
  std::array<std::pair<util::Pose, float>, kIterations> proposed_commands;

  std::normal_distribution<> translational_noise_dist(
      0.0f, params::kProposedTranslationStdDev);
  std::normal_distribution<> rotational_noise_dist(
      0.0f, params::kProposedRotationStdDev);

  const std::array<util::Pose, 2> special_poses = {{{0, 0, 1}, {0, 0, -1}}};

  const auto generate_special =
      [&special_poses,
       &commanded_velocity](const int& i) -> std::pair<util::Pose, float> {
    NP_CHECK(static_cast<size_t>(i) < special_poses.size());
    const auto& special = special_poses[i];
    const auto delta_pose = commanded_velocity - special;
    const float cost = math_util::Sq(delta_pose.tra.lpNorm<1>()) +
                       math_util::Sq(delta_pose.rot);
    return std::make_pair(special, cost);
  };
  const auto generate_random =
      [this,
       &translational_noise_dist,
       &rotational_noise_dist]() -> std::pair<util::Pose, float> {
    const float translational_noise = translational_noise_dist(random_gen_);
    const float rotational_noise = rotational_noise_dist(random_gen_);
    return std::make_pair(
        util::Pose(translational_noise, 0, rotational_noise),
        math_util::Sq(translational_noise) + math_util::Sq(rotational_noise));
  };

  for (int i = 0; i < kIterations; ++i) {
    const auto delta = (static_cast<size_t>(i) < special_poses.size())
                           ? generate_special(i)
                           : generate_random();
    const util::Pose proposed_command =
        ApplyCommandLimits(commanded_velocity + delta.first);
    if (!IsCommandColliding(proposed_command, rollout_duration, robot_radius)) {
      const float cost = delta.second;
      //      ROS_INFO("Proposed command: (%f, %f), %f cost %f",
      //               proposed_command.tra.x(), proposed_command.tra.y(),
      //               proposed_command.rot, cost);
      proposed_commands[i] = {proposed_command, cost};
    } else {
      //      ROS_INFO("Proposed command: (%f, %f), %f (colliding)",
      //               proposed_command.tra.x(), proposed_command.tra.y(),
      //               proposed_command.rot);
      proposed_commands[i] = {proposed_command,
                              std::numeric_limits<float>::max()};
    }
  }

  int min_index = 0;
  for (int i = 1; i < kIterations; ++i) {
    if (proposed_commands[i].second < proposed_commands[min_index].second) {
      min_index = i;
    }
  }
  const auto& best = proposed_commands[min_index];
  if (best.second < std::numeric_limits<float>::max()) {
    return best.first;
  }
  return zero_command;
}

}  // namespace obstacle_avoidance
}  // namespace cs
