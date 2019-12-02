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

#include <vector>

#include "cs/path_finding/path_finder.h"
#include "cs/util/constants.h"
#include "cs/util/geometry.h"
#include "cs/util/math_util.h"

namespace cs {
namespace path_finding {

template <typename T>

class RRT : public PathFinder {
 private:
  std::default_random_engine generator;

  bool IsLineColliding(const util::Map& dynamic_map,
                       const Eigen::Vector2f& p1,
                       const Eigen::Vector2f& p2) const {
    for (const auto& w : map_.walls) {
      if (geometry::CheckLineLineCollision(w.p1, w.p2, p1, p2)) {
        return true;
      }
    }
    for (const auto& w : dynamic_map.walls) {
      if (geometry::CheckLineLineCollision(w.p1, w.p2, p1, p2)) {
        return true;
      }
    }
    return false;
  }

  struct TreePoint {
    static constexpr int kRootParent = -1;
    Eigen::Vector2f point;
    int parent;

    TreePoint() = delete;
    TreePoint(const Eigen::Vector2f& point, const int parent)
        : point(point), parent(parent) {}
  };

  struct Tree {
    std::vector<Eigen::Vector2f> points;

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
      TreePoint const& current_point = points[goal_idx];
      while (current_point.parent != TreePoint::kRootParent) {
        TreePoint const& parent_point = points[current_point.parent];
        p.cost += (parent_point.point - current_point.point).norm();
        p.waypoints.push_back(parent_point.point);
        current_point = parent_point;
      }
      return p;
    }
  };

  int AddPoint(const util::Map& dynamic_map,
               Tree* tree,
               const Eigen::Vector2f& steering_point) {
    static constexpr float kDeltaQ = 0.05;  // meters
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
    static constexpr float kDistanceToGoal = 0.2;  // meters.
    return (goal - point).squaredNorm() < math_util::Sq(kDistanceToGoal);
  }

  Path2d GenerateNewPath(const util::Map& dynamic_map,
                         const Eigen::Vector2f& start,
                         const Eigen::Vector2f& goal) {
    static constexpr int kMaxIterations = 1000;
    Tree tree(start);
    static constexpr float kExtraSampleArea = 1;  // meter
    const float x_min = std::min(start.x(), goal.x()) - kExtraSampleArea;
    const float x_max = std::max(start.x(), goal.x()) + kExtraSampleArea;
    const float y_min = std::min(start.y(), goal.y()) - kExtraSampleArea;
    const float y_max = std::max(start.y(), goal.y()) + kExtraSampleArea;

    std::uniform_real_distribution<float> x_dist(x_min, x_max);
    std::uniform_real_distribution<float> y_dist(y_min, y_max);

    for (int i = 0; i < kMaxIterations; ++i) {
      const float x_sample = x_dist(generator);
      const float y_sample = y_dist(generator);
      const Eigen::Vector2f sample(x_sample, y_sample);
      const int added_point_idx = AddPoint(dynamic_map, &tree, sample);
      if (TreePoint::kRootParent == added_point_idx) {
        continue;
      }
      const Eigen::Vector2f& added_point = tree.points[added_point_idx].point;
      if (IsNearGoal(goal, added_point)) {
        return tree.UnwindPath(added_point_idx);
      }
    }
    return {};
  }

  bool IsPathCollisionFree(const util::Map& dynamic_map,
                           const Path2d& path) const {
    if (!path.IsValid()) {
      return false;
    }
    for (size_t i = 0; i < path.waypoints.size() - 1; ++i) {
      const auto& curr = path.waypoints[i];
      const auto& next = path.waypoints[i + 1];
      if (IsLineColliding(dynamic_map, curr, next)) {
        return true;
      }
    }
    return false;
  }

 public:
  explicit RRT(const util::Map& map) : PathFinder(map) {}

  Path2d FindPath(const util::Map& dynamic_map,
                  const Eigen::Vector2f& start,
                  const Eigen::Vector2f& goal) override {
    // Drive straight to goal.
    if (!IsLineColliding(dynamic_map, start, goal)) {
      return UsePrevPathOrUpdate(Path2d({start, goal}, (start - goal).norm()));
    }

    const Path2d new_path = GenerateNewPath(dynamic_map, start, goal);
    if (!new_path.IsValid()) {
    }
    return {};
  }
};

}  // namespace path_finding
}  // namespace cs
