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
#include "cs/util/geometry.h"
#include "cs/util/laser_scan.h"
#include "cs/util/math_util.h"
#include "cs/util/params.h"
#include "cs/util/visualization.h"

namespace cs {
namespace obstacle_avoidance {

// namespace params {
//// CONFIG_FLOAT(kProposedTranslationStdDev, "od.kProposedTranslationStdDev");
//// CONFIG_FLOAT(kProposedRotationStdDev, "od.kProposedRotationStdDev");
//// CONFIG_FLOAT(kMinDistanceThreshold, "od.kMinDistanceThreshold");
// static constexpr float kMinDistanceThreshold = 0.05;
// static constexpr float kProposedTranslationStdDev = 1.0;
// static constexpr float kProposedRotationStdDev = 5;
//
//// CONFIG_FLOAT(kMaxTraVel, "limits.kMaxTraVel");
//// CONFIG_FLOAT(kMaxRotVel, "limits.kMaxRotVel");
//// CONFIG_FLOAT(kMaxTraAcc, "limits.kMaxTraAcc");
//// CONFIG_FLOAT(kMaxRotAcc, "limits.kMaxRotAcc");
// static constexpr float kMaxTraAcc = 3;
// static constexpr float kMaxTraVel = 1;
// static constexpr float kMaxRotAcc = 2;
// static constexpr float kMaxRotVel = 1;
//
//// CONFIG_FLOAT(kOdomFilteringPriorBias, "od.kOdomFilteringPriorBias");
//// CONFIG_FLOAT(kThresholdRotateInPlace, "od.kThresholdRotateInPlace");
// static constexpr float kOdomFilteringPriorBias = 0.7;
// static constexpr float kThresholdRotateInPlace = 0.9;
// static constexpr float kTranslationCostScaleFactor = 1000;
//}  // namespace params

ObstacleDetector::ObstacleDetector(const util::Map& map)
    : map_(map),
      estimated_pose_(0, 0, 0),
      odom_velocity_(0, 0, 0),
      prior_commanded_velocity_(0, 0, 0),
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

  const auto in_same_cluster = [&start_v](const Eigen::Vector2f& prev,
                                          const Eigen::Vector2f& curr) -> bool {
    static constexpr float kMaxDistanceBetweenReadings = 0.01;
    static constexpr float kMinDistanceBetweenReadingsToReasonAngle = 0.001;

    const Eigen::Vector2f delta = (curr - prev);
    if (delta.squaredNorm() > math_util::Sq(kMaxDistanceBetweenReadings)) {
      return false;
    }

    if (start_v == prev) {
      return true;
    }

    if (delta.squaredNorm() <
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
    NP_CHECK(prev_idx < points.size());
    NP_CHECK(i < points.size());
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

util::Twist ObstacleDetector::EstimateCurrentVelocity() const {
  return odom_velocity_ * (1 - params::kOdomFilteringPriorBias) +
         prior_commanded_velocity_ * (params::kOdomFilteringPriorBias);
}

void ObstacleDetector::UpdateOdom(const util::Pose& pose,
                                  const util::Twist& velocity) {
  estimated_pose_ = pose;
  odom_velocity_ = velocity;
}

void ObstacleDetector::UpdateCommand(
    const util::Twist& prior_commanded_velocity) {
  prior_commanded_velocity_ = prior_commanded_velocity;
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

    std::vector<Eigen::Vector2f> cluster_points;
    for (size_t i = cluster_start; i <= cluster_end; ++i) {
      cluster_points.push_back(non_map_points[i]);
    }

    if (kDebug) {
      ROS_INFO("Cluster size: %zu", (cluster_end - cluster_start + 1));
    }

    const auto color = visualization::IndexToDistinctRBG(num_clusters);
    visualization::PointsToSpheres(cluster_points,
                                   "map",
                                   "non_points_ns",
                                   &new_markers,
                                   std::get<0>(color),
                                   std::get<1>(color),
                                   std::get<2>(color),
                                   0,
                                   0.01);
    new_markers.markers.push_back(
        visualization::ToLine(dynamic_walls_.back().p1,
                              dynamic_walls_.back().p2,
                              "map",
                              "colored_wall_ns",
                              num_clusters,
                              std::get<0>(color),
                              std::get<1>(color),
                              std::get<2>(color),
                              0.01));

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
  estimated_pose_ = observation_pose;
}

void ObstacleDetector::DrawDynamic(ros::Publisher* pub) const {
  // ROS_INFO("Obstacle detector found: %zu obstacles", dynamic_walls_.size());
  pub->publish(
      visualization::DrawWalls(dynamic_walls_, "map", "dynamic_walls_ns"));
}

const std::vector<util::Wall>& ObstacleDetector::GetDynamicWalls() const {
  return dynamic_walls_;
}

bool ObstacleDetector::IsCommandColliding(const util::Twist& commanded_velocity,
                                          const float rollout_duration,
                                          const float robot_radius,
                                          const float safety_margin) const {
  static constexpr bool kDebug = false;
  const TrajectoryRollout tr(estimated_pose_,
                             EstimateCurrentVelocity(),
                             commanded_velocity,
                             rollout_duration);
  for (const auto& w : dynamic_walls_) {
    if (tr.IsColliding(w, robot_radius, safety_margin)) {
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
    if (tr.IsColliding(w, robot_radius, safety_margin)) {
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

util::Twist ObstacleDetector::ApplyCommandLimits(
    util::Twist c, const float& time_delta) const {
  static constexpr bool kDebug = false;
  NP_CHECK(time_delta >= 0);
  if (kDebug) {
    ROS_INFO("Time delta: %f", time_delta);
  }

  const util::Twist estimated_velocity = EstimateCurrentVelocity();

  if (kDebug) {
    ROS_INFO("estimated_velocity (%f, %f), %f",
             estimated_velocity.tra.x(),
             estimated_velocity.tra.y(),
             estimated_velocity.rot);
  }

  if (time_delta == 0) {
    return estimated_velocity;
  }
  if (kDebug) {
    ROS_INFO("Raw command: (%f, %f), %f", c.tra.x(), c.tra.y(), c.rot);
  }

  // Cap translational velocity.
  if (c.tra.squaredNorm() > math_util::Sq(params::kMaxTraVel)) {
    c.tra = c.tra.normalized() * params::kMaxTraVel;
  }
  // Cap rotational velocity.
  if (fabs(c.rot) > params::kMaxRotVel) {
    c.rot = math_util::Sign(c.rot) * params::kMaxRotVel;
  }

  if (kDebug) {
    ROS_INFO("Vel capped command: (%f, %f), %f", c.tra.x(), c.tra.y(), c.rot);
  }

  // m/s - m/s -> m/s / s = m/s^2
  const util::Twist acceleration = (c - estimated_velocity) / time_delta;

  if (kDebug) {
    ROS_INFO("Acceleration (%f, %f), %f",
             acceleration.tra.x(),
             acceleration.tra.y(),
             acceleration.rot);
  }

  // Cap translational acceleration.
  if (acceleration.tra.squaredNorm() > math_util::Sq(params::kMaxTraAcc)) {
    // m/s = m/s + m/s^2 * s
    c.tra = estimated_velocity.tra +
            acceleration.tra.normalized() * params::kMaxTraAcc * time_delta;
  }

  if (kDebug) {
    ROS_INFO("Acc capped command: (%f, %f), %f", c.tra.x(), c.tra.y(), c.rot);
  }

  // Cap rotational acceleration.
  if (fabs(acceleration.rot) > math_util::Sq(params::kMaxRotAcc)) {
    // rad/s = rad/s + rad/s^2 * s
    c.rot = estimated_velocity.rot +
            math_util::Sign(acceleration.rot) * params::kMaxRotAcc * time_delta;
  }

  if (kDebug) {
    ROS_INFO(
        "Acc capped rot command: (%f, %f), %f", c.tra.x(), c.tra.y(), c.rot);
  }

  return c;
}

bool ObstacleDetector::StartedInCollision(const float robot_radius) const {
  const auto& center = estimated_pose_.tra;
  for (const auto& w : dynamic_walls_) {
    const auto projected_point =
        geometry::ProjectPointOntoLineSegment(center, w.p1, w.p2);
    if ((center - projected_point).squaredNorm() <
        math_util::Sq(robot_radius)) {
      return true;
    }
  }

  for (const auto& w : map_.walls) {
    const auto projected_point =
        geometry::ProjectPointOntoLineSegment(center, w.p1, w.p2);
    if ((center - projected_point).squaredNorm() <
        math_util::Sq(robot_radius)) {
      return true;
    }
  }
  return false;
}

util::Twist ObstacleDetector::MakeCommandSafe(util::Twist commanded_velocity,
                                              const float time_delta,
                                              const float rollout_duration,
                                              const float robot_radius,
                                              const float safety_margin) {
  ROS_INFO("Initial command: (%f, %f), %f",
           commanded_velocity.tra.x(),
           commanded_velocity.tra.y(),
           commanded_velocity.rot);
  commanded_velocity = ApplyCommandLimits(commanded_velocity, time_delta);

  // if (StartedInCollision(robot_radius)) {
  //   ROS_WARN("Started in collision!");
  //   return {0, 0, 1};
  // }
  const auto est_vel = EstimateCurrentVelocity();
  ROS_INFO("Limited command: (%f, %f), %f",
           commanded_velocity.tra.x(),
           commanded_velocity.tra.y(),
           commanded_velocity.rot);
  ROS_INFO("Estimated velocity: (%f, %f), %f",
           est_vel.tra.x(),
           est_vel.tra.y(),
           est_vel.rot);
  ROS_INFO("Time delta: %f", time_delta);
  static constexpr bool kDebug = false;
  if (kDebug) {
    ROS_INFO("Current position: (%f, %f), %f",
             estimated_pose_.tra.x(),
             estimated_pose_.tra.y(),
             estimated_pose_.rot);
    ROS_INFO("Current velocity: (%f, %f), %f",
             odom_velocity_.tra.x(),
             odom_velocity_.tra.y(),
             odom_velocity_.rot);
    ROS_INFO("Current command: (%f, %f), %f",
             commanded_velocity.tra.x(),
             commanded_velocity.tra.y(),
             commanded_velocity.rot);
  }
  if (!IsCommandColliding(
          commanded_velocity, rollout_duration, robot_radius, safety_margin)) {
    return commanded_velocity;
  }

  static constexpr int kIterations = 50;
  std::array<std::pair<util::Twist, float>, kIterations> proposed_commands;

  std::normal_distribution<> translational_noise_dist(
      0.0f, params::kProposedTranslationStdDev);
  std::normal_distribution<> rotational_noise_dist(
      0.0f, params::kProposedRotationStdDev);

  const std::array<util::Twist, 5> special_poses = {
      {{0, 0, 0},
       {0, 0, params::kMaxRotVel},
       {0, 0, -params::kMaxRotVel},
       {est_vel.tra, params::kMaxRotVel},
       {est_vel.tra, -params::kMaxRotVel}}};

  const auto generate_special =
      [&special_poses,
       &commanded_velocity](const int& i) -> std::pair<util::Twist, float> {
    NP_CHECK(static_cast<size_t>(i) < special_poses.size());
    const auto& special = special_poses[i];
    const auto delta_pose = commanded_velocity - special;
    const float cost = math_util::Sq(delta_pose.tra.lpNorm<1>() *
                                     params::kTranslationCostScaleFactor) +
                       fabs(delta_pose.rot);
    return std::make_pair(special, cost);
  };
  const auto generate_random =
      [this,
       &translational_noise_dist,
       &rotational_noise_dist]() -> std::pair<util::Twist, float> {
    const float translational_noise = translational_noise_dist(random_gen_);
    const float rotational_noise = rotational_noise_dist(random_gen_);
    return std::make_pair(util::Twist(translational_noise, 0, rotational_noise),
                          math_util::Sq(translational_noise *
                                        params::kTranslationCostScaleFactor) +
                              fabs(rotational_noise));
  };

  for (int i = 0; i < kIterations; ++i) {
    const auto delta = (static_cast<size_t>(i) < special_poses.size())
                           ? generate_special(i)
                           : generate_random();
    const util::Twist proposed_command =
        ApplyCommandLimits(commanded_velocity + delta.first, time_delta);
    if (!IsCommandColliding(
            proposed_command, rollout_duration, robot_radius, safety_margin)) {
      const float cost = delta.second;
      if (static_cast<size_t>(i) < special_poses.size()) {
        ROS_INFO("Special command: (%f, %f), %f cost %f (non-colliding)",
                 proposed_command.tra.x(),
                 proposed_command.tra.y(),
                 proposed_command.rot,
                 cost);
      }
      proposed_commands[i] = {proposed_command, cost};
    } else {
      if (static_cast<size_t>(i) < special_poses.size()) {
        ROS_INFO("Proposed command: (%f, %f), %f (colliding)",
                 proposed_command.tra.x(),
                 proposed_command.tra.y(),
                 proposed_command.rot);
      }
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
    ROS_INFO("Final cmd velocity: (%f, %f), %f",
             best.first.tra.x(),
             best.first.tra.y(),
             best.first.rot);
    return best.first;
  }

  const util::Twist est_current_velocity = EstimateCurrentVelocity();
  if (est_current_velocity.tra.squaredNorm() +
          math_util::Sq(est_current_velocity.rot) <
      math_util::Sq(params::kThresholdRotateInPlace)) {
    ROS_INFO("Rotate in place!!");
    return ApplyCommandLimits({0, 0, params::kMaxRotVel / 2}, time_delta);
  }
  return {0, 0, 0};
}

}  // namespace obstacle_avoidance
}  // namespace cs
