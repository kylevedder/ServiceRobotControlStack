// Copyright 2019 kvedder@seas.upenn.edu
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

#include "cs/path_finding/path_finder.h"

#include <algorithm>
#include <limits>
#include <string>

#include "config_reader/macros.h"
#include "cs/util/constants.h"
#include "shared/math/geometry.h"

namespace cs {
namespace path_finding {

namespace params {
CONFIG_FLOAT(switch_historesis_threshold,
             "path_finding.switch_historesis_threshold");
CONFIG_FLOAT(goal_delta_change, "path_finding.goal_delta_change");
CONFIG_FLOAT(max_distance_off_path, "path_finding.max_distance_off_path");
};  // namespace params

Path2f PathFinder::UsePrevPathOrUpdate(const util::Map& dynamic_map,
                                       const Path2f& proposed_path) {
  static constexpr bool kDebug = false;
  NP_CHECK(!proposed_path.IsValid() ||
           !IsPathColliding(dynamic_map, proposed_path));

  // Old path invalid.
  if (!prev_path_.IsValid()) {
    if (kDebug) {
      ROS_INFO("Prev path invalid");
    }
    prev_path_ = proposed_path;
    return proposed_path;
  }

  // Old path colliding.
  if (IsPathColliding(dynamic_map, prev_path_)) {
    if (kDebug) {
      ROS_INFO("Prev path colliding");
    }
    prev_path_ = proposed_path;
    return proposed_path;
  }

  // New path invalid.
  if (!proposed_path.IsValid()) {
    return prev_path_;
  }

  const float distance_between_goals_sq =
      (prev_path_.waypoints.back() - proposed_path.waypoints.back())
          .squaredNorm();

  // Goal moved.
  if (distance_between_goals_sq >
      math_util::Sq(params::CONFIG_goal_delta_change)) {
    if (kDebug) {
      ROS_INFO("Goal Changed");
    }
    prev_path_ = proposed_path;
    return proposed_path;
  }

  const Eigen::Vector2f robot_position = proposed_path.waypoints.front();
  const float distance_from_first_segment_sq =
      (geometry::ProjectPointOntoLine(
           robot_position, prev_path_.waypoints[0], prev_path_.waypoints[1]) -
       robot_position)
          .squaredNorm();

  // Too far away from old path.
  if (distance_from_first_segment_sq >
      math_util::Sq(params::CONFIG_max_distance_off_path)) {
    prev_path_ = proposed_path;
    return proposed_path;
  }

  // New path is significantly less expensive.
  if (params::CONFIG_switch_historesis_threshold + proposed_path.cost <
      prev_path_.cost) {
    if (kDebug) {
      ROS_INFO("Proposed path better");
    }

    if (prev_path_.waypoints.size() >= 3) {
      prev_path_.waypoints.erase(prev_path_.waypoints.begin());
      const float distance_from_new_first_segment_sq =
          (geometry::ProjectPointOntoLine(robot_position,
                                          prev_path_.waypoints[0],
                                          prev_path_.waypoints[1]) -
           robot_position)
              .squaredNorm();
      if (distance_from_new_first_segment_sq <=
          math_util::Sq(params::CONFIG_max_distance_off_path)) {
        return prev_path_;
      }
    }

    prev_path_ = proposed_path;
    return proposed_path;
  }
  return prev_path_;
}

bool PathFinder::IsLineColliding(const util::Map& dynamic_map,
                                 const Eigen::Vector2f& p1,
                                 const Eigen::Vector2f& p2) const {
  for (const auto& w : map_.walls) {
    if (geometry::MinDistanceLineLine(w.p1, w.p2, p1, p2) <=
        (robot_radius_ + safety_margin_)) {
      return true;
    }
  }
  for (const auto& w : dynamic_map.walls) {
    if (geometry::MinDistanceLineLine(w.p1, w.p2, p1, p2) <=
        (robot_radius_ + safety_margin_)) {
      return true;
    }
  }
  return false;
}

bool PathFinder::IsPathColliding(const util::Map& dynamic_map,
                                 const Path2f& path) const {
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

Path2f SlicePath(Path2f path, const size_t end_idx) {
  NP_CHECK(path.waypoints.size() > end_idx)
  if (end_idx > 2) {
    path.waypoints.erase(path.waypoints.begin() + 1,
                         path.waypoints.begin() + (end_idx - 1));
  }
  return path;
}

Path2f PathFinder::SmoothPath(const util::Map& dynamic_map, Path2f path) const {
  if (path.waypoints.size() <= 2 || IsPathColliding(dynamic_map, path)) {
    return path;
  }
  const auto& start = path.waypoints.front();
  for (size_t i = 2; i < path.waypoints.size(); ++i) {
    const auto& intermediate = path.waypoints[i];
    if (IsLineColliding(dynamic_map, start, intermediate)) {
      return SlicePath(std::move(path), i);
    }
  }
  path.waypoints = {path.waypoints.front(), path.waypoints.back()};
  return path;
}

}  // namespace path_finding
}  // namespace cs
