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
               const float& safety_margin);

  Path2d FindPath(const util::Map& dynamic_map,
                  const Eigen::Vector2f& start,
                  const Eigen::Vector2f& goal,
                  ros::Publisher* pub) override;

 private:
  const float& robot_radius;
  const float& safety_margin;
  struct TreePoint {
    static constexpr int kRootParent = -1;
    Eigen::Vector2f point;
    int parent;

    TreePoint() = delete;
    TreePoint(const Eigen::Vector2f& point, const int parent);
  };

  struct Tree {
    std::vector<TreePoint> points;

    explicit Tree(const Eigen::Vector2f& start)
        : points({TreePoint(start, TreePoint::kRootParent)}) {}

    int FindClosestPoint(const Eigen::Vector2f& steering_point);

    Path2d UnwindPath(const int goal_idx) const;
  };
  std::default_random_engine generator;

  visualization_msgs::Marker DrawTree(const cs::path_finding::RRT::Tree& tree,
                                      const std::string& frame_id,
                                      const std::string& ns);

  bool IsLineColliding(const util::Map& dynamic_map,
                       const Eigen::Vector2f& p1,
                       const Eigen::Vector2f& p2) const;

  int AddPoint(const util::Map& dynamic_map,
               Tree* tree,
               const Eigen::Vector2f& steering_point);

  bool IsNearGoal(const Eigen::Vector2f& goal, const Eigen::Vector2f& point);

  Path2d SmoothPath(const util::Map& dynamic_map, Path2d path);

  Path2d GenerateNewPath(const util::Map& dynamic_map,
                         const Eigen::Vector2f& start,
                         const Eigen::Vector2f& goal,
                         ros::Publisher* pub);

  bool IsPathColliding(const util::Map& dynamic_map,
                       const Path2d& path) const override;
};

}  // namespace path_finding
}  // namespace cs
