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

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <vector>

#include "cs/util/map.h"
#include "cs/util/pose.h"

namespace cs {
namespace path_finding {

template <typename Position, typename Cost>
struct Path {
  std::vector<Position> waypoints;
  Cost cost;

  Path() : waypoints(), cost(0) {}
  Path(const std::vector<Position>& waypoints, const Cost& cost)
      : waypoints(waypoints), cost(cost) {}

  bool IsValid() const { return !waypoints.empty(); }
};

using Path2d = Path<Eigen::Vector2f, float>;

class PathFinder {
 protected:
  const util::Map& map_;
  Path2d prev_path_;
  double prev_path_time_;

  virtual bool IsPathColliding(const util::Map& dynamic_map,
                               const Path2d& path) const = 0;

  Path2d UsePrevPathOrUpdate(const util::Map& dynamic_map,
                             const Path2d& proposed_path) {
    static constexpr float kHistoresisThreshold = 0.4;  // meters

    NP_CHECK(!proposed_path.IsValid() ||
             !IsPathColliding(dynamic_map, proposed_path));

    if (!prev_path_.IsValid()) {
      ROS_INFO("Prev path invalid");
      prev_path_ = proposed_path;
      return proposed_path;
    }

    if (IsPathColliding(dynamic_map, prev_path_)) {
      ROS_INFO("Prev path colliding");
      prev_path_ = proposed_path;
      return proposed_path;
    }

    const float distance_between_starts =
        (proposed_path.waypoints.front() - prev_path_.waypoints.front())
            .squaredNorm();

    if (proposed_path.IsValid() &&
        (kHistoresisThreshold + proposed_path.cost < prev_path_.cost ||
         distance_between_starts > math_util::Sq(kHistoresisThreshold))) {
      ROS_INFO("Proposed path better");
      prev_path_ = proposed_path;
      return proposed_path;
    }
    return prev_path_;
  }

 public:
  explicit PathFinder(const util::Map& map) : map_(map) {}

  virtual Path2d FindPath(const util::Map& dynamic_map,
                          const Eigen::Vector2f& start,
                          const Eigen::Vector2f& goal,
                          ros::Publisher* pub) = 0;
};

}  // namespace path_finding
}  // namespace cs
