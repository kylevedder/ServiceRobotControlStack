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

#include <algorithm>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "cs/path_finding/path_finder.h"
#include "cs/util/constants.h"
#include "cs/util/geometry.h"
#include "cs/util/math_util.h"
#include "cs/util/visualization.h"

namespace cs {
namespace path_finding {

class RRT : public PathFinder {
 public:
  explicit RRT(const util::Map& map,
               const float& robot_radius,
               const float& safety_margin)
      : PathFinder(map),
        robot_radius(robot_radius),
        safety_margin(safety_margin) {}

  Path2d FindPath(const util::Map& dynamic_map,
                  const Eigen::Vector2f& start,
                  const Eigen::Vector2f& goal,
                  ros::Publisher* pub) override {
    // Drive straight to goal.
    if (!IsLineColliding(dynamic_map, start, goal)) {
      return UsePrevPathOrUpdate(dynamic_map,
                                 Path2d({start, goal}, (start - goal).norm()));
    }

    return UsePrevPathOrUpdate(dynamic_map,
                               GenerateNewPath(dynamic_map, start, goal, pub));
  }

 private:
  const float& robot_radius;
  const float& safety_margin;
  struct TreePoint {
    static constexpr int kRootParent = -1;
    Eigen::Vector2f point;
    int parent;

    TreePoint() = delete;
    TreePoint(const Eigen::Vector2f& point, const int parent)
        : point(point), parent(parent) {}
  };

  struct Tree {
    std::vector<TreePoint> points;

    explicit Tree(const Eigen::Vector2f& start)
        : points({TreePoint(start, TreePoint::kRootParent)}) {}

    int FindClosestPoint(const Eigen::Vector2f& steering_point) {
      float min_sq_distance = std::numeric_limits<float>::max();
      float min_i = 0;
      for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        const float i_sq_dist =
            (points[i].point - steering_point).squaredNorm();
        if (i_sq_dist < min_sq_distance) {
          min_i = i;
          min_sq_distance = i_sq_dist;
        }
      }

      return min_i;
    }

    Path2d UnwindPath(const int goal_idx) const {
      Path2d p;
      p.waypoints.push_back(points[goal_idx].point);
      TreePoint current_point = points[goal_idx];
      while (current_point.parent != TreePoint::kRootParent) {
        TreePoint parent_point = points[current_point.parent];
        p.cost += (parent_point.point - current_point.point).norm();
        p.waypoints.push_back(parent_point.point);
        current_point = parent_point;
      }
      return p;
    }
  };
  std::default_random_engine generator;

  visualization_msgs::Marker DrawTree(const cs::path_finding::RRT::Tree& tree,
                                      const std::string& frame_id,
                                      const std::string& ns) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;

    for (const auto& tree_point : tree.points) {
      if (tree_point.parent == cs::path_finding::RRT::TreePoint::kRootParent) {
        continue;
      }
      const auto& parent_tree_point = tree.points[tree_point.parent];
      geometry_msgs::Point p1;
      p1.x = tree_point.point.x();
      p1.y = tree_point.point.y();
      geometry_msgs::Point p2;
      p2.x = parent_tree_point.point.x();
      p2.y = parent_tree_point.point.y();
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }

    return marker;
  }

  bool IsLineColliding(const util::Map& dynamic_map,
                       const Eigen::Vector2f& p1,
                       const Eigen::Vector2f& p2) const {
    for (const auto& w : map_.walls) {
      if (geometry::CheckLineLineCollision(w.p1, w.p2, p1, p2)) {
        return true;
      }

      if (geometry::MinDistanceLineLine(w.p1, w.p2, p1, p2) <
          (robot_radius + safety_margin)) {
        return true;
      }
    }
    for (const auto& w : dynamic_map.walls) {
      if (geometry::CheckLineLineCollision(w.p1, w.p2, p1, p2)) {
        return true;
      }

      if (geometry::MinDistanceLineLine(w.p1, w.p2, p1, p2) <
          (robot_radius + safety_margin)) {
        return true;
      }
    }
    return false;
  }

  int AddPoint(const util::Map& dynamic_map,
               Tree* tree,
               const Eigen::Vector2f& steering_point) {
    static constexpr float kDeltaQ = 1;  // meters
    const int closest_idx = tree->FindClosestPoint(steering_point);
    NP_CHECK(closest_idx < static_cast<int>(tree->points.size()));
    const Eigen::Vector2f& tree_point = tree->points[closest_idx].point;
    const Eigen::Vector2f norm_delta =
        (steering_point - tree_point).normalized();
    const Eigen::Vector2f final_point = norm_delta * kDeltaQ + tree_point;

    if (!IsLineColliding(dynamic_map, tree_point, final_point)) {
      tree->points.push_back({final_point, closest_idx});
      return static_cast<int>(tree->points.size()) - 1;
    }
    return TreePoint::kRootParent;
  }

  bool IsNearGoal(const Eigen::Vector2f& goal, const Eigen::Vector2f& point) {
    static constexpr float kDistanceToGoal = 0.4;  // meters.
    return (goal - point).squaredNorm() < math_util::Sq(kDistanceToGoal);
  }

  Path2d SmoothPath(const util::Map& dynamic_map, Path2d path) {
    if (!path.IsValid()) {
      return path;
    }

    NP_CHECK(!IsPathColliding(dynamic_map, path));
    for (size_t i = 2; i < path.waypoints.size(); ++i) {
      if (IsLineColliding(dynamic_map, path.waypoints[0], path.waypoints[i])) {
        if (i > 3) {
          path.waypoints.erase(path.waypoints.begin() + 1,
                               path.waypoints.begin() + (i - 1));
        }
        break;
      }
    }
    NP_CHECK(!IsPathColliding(dynamic_map, path));
    return path;
  }

  Path2d GenerateNewPath(const util::Map& dynamic_map,
                         const Eigen::Vector2f& start,
                         const Eigen::Vector2f& goal,
                         ros::Publisher* pub) {
    static constexpr int kMaxIterations = 1000;
    static constexpr float kGoalBias = 0.1;
    Tree tree(start);
    static constexpr float kExtraSampleArea = 1;  // meter

    float x_min = std::min(start.x(), goal.x()) - kExtraSampleArea;
    float x_max = std::max(start.x(), goal.x()) + kExtraSampleArea;
    float y_min = std::min(start.y(), goal.y()) - kExtraSampleArea;
    float y_max = std::max(start.y(), goal.y()) + kExtraSampleArea;

    for (const auto& w : map_.walls) {
      x_min = std::min({x_min, w.p1.x(), w.p2.x()});
      x_max = std::max({x_max, w.p1.x(), w.p2.x()});
      y_min = std::min({y_min, w.p1.y(), w.p2.y()});
      y_max = std::max({y_max, w.p1.y(), w.p2.y()});
    }

    std::uniform_real_distribution<float> x_dist(x_min, x_max);
    std::uniform_real_distribution<float> y_dist(y_min, y_max);
    std::uniform_real_distribution<float> goal_bias_dist(0, 1);

    for (int i = 0; i < kMaxIterations; ++i) {
      const bool choose_goal = (goal_bias_dist(generator) <= kGoalBias);
      const float x_sample = (choose_goal) ? goal.x() : x_dist(generator);
      const float y_sample = (choose_goal) ? goal.y() : y_dist(generator);
      const Eigen::Vector2f sample(x_sample, y_sample);
      const int added_point_idx = AddPoint(dynamic_map, &tree, sample);
      if (TreePoint::kRootParent == added_point_idx) {
        continue;
      }
      const Eigen::Vector2f& added_point = tree.points[added_point_idx].point;
      if (IsNearGoal(goal, added_point)) {
        if (pub != nullptr) {
          pub->publish(DrawTree(tree, "map", "tree"));
        }
        auto path = tree.UnwindPath(added_point_idx);
        std::reverse(path.waypoints.begin(), path.waypoints.end());
        return SmoothPath(dynamic_map, path);
      }
    }
    if (pub != nullptr) {
      pub->publish(DrawTree(tree, "map", "tree"));
    }
    ROS_WARN("Reached max iterations without finding goal!");
    return {};
  }

  bool IsPathColliding(const util::Map& dynamic_map,
                       const Path2d& path) const override {
    if (path.waypoints.size() < 2) {
      return true;
    }
    for (size_t i = 0; i < path.waypoints.size() - 1; ++i) {
      NP_CHECK(i < path.waypoints.size());
      NP_CHECK(i + 1 < path.waypoints.size());
      const auto& curr = path.waypoints[i];
      const auto& next = path.waypoints[i + 1];
      if (IsLineColliding(dynamic_map, curr, next)) {
        return true;
      }
    }
    return false;
  }
};

}  // namespace path_finding
}  // namespace cs
