#pragma once
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

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <vector>

#include "cs/util/map.h"
#include "cs/util/pose.h"

#include "config_reader/macros.h"

namespace cs {
namespace path_finding {

template <typename Position, typename Cost>
struct Path {
  std::vector<Position> waypoints;
  Cost cost;

  Path() : waypoints(), cost(0) {}
  Path(const std::vector<Position>& waypoints, const Cost& cost)
      : waypoints(waypoints), cost(cost) {}

  Path(const Path& p) : waypoints(p.waypoints), cost(p.cost) {}
  Path(Path&& p) : waypoints(std::move(p.waypoints)), cost(std::move(p.cost)) {}
  Path& operator=(const Path& p) {
    this->waypoints = p.waypoints;
    this->cost = p.cost;
    return *this;
  };
  Path& operator=(Path&& p) {
    this->waypoints = std::move(p.waypoints);
    this->cost = std::move(p.cost);
    return *this;
  };

  bool IsValid() const { return !waypoints.empty(); }

  template <typename Transform>
  Path TransformPath(const Transform& t) const {
    Path path = *this;
    for (auto& p : path.waypoints) {
      p = t * p;
    }
    return path;
  }
};

using Path2f = Path<Eigen::Vector2f, float>;

class PathFinder {
 protected:
  const util::Map& map_;
  const float& robot_radius_;
  const float& safety_margin_;
  Path2f prev_path_;
  double prev_path_time_;

  bool IsLineColliding(const util::Map& dynamic_map,
                       const Eigen::Vector2f& p1,
                       const Eigen::Vector2f& p2) const;

  bool IsPathColliding(const util::Map& dynamic_map, const Path2f& path) const;

  Path2f UsePrevPathOrUpdate(const util::Map& dynamic_map,
                             const Path2f& proposed_path);

  Path2f SmoothPath(const Eigen::Vector2f& start,
                    const util::Map& dynamic_map,
                    Path2f path) const;

 public:
  PathFinder(const util::Map& map,
             const float& robot_radius,
             const float& safety_margin)
      : map_(map), robot_radius_(robot_radius), safety_margin_(safety_margin) {}

  virtual Path2f FindPath(const util::Map& dynamic_map,
                          const Eigen::Vector2f& start,
                          const Eigen::Vector2f& goal) = 0;
};

}  // namespace path_finding
}  // namespace cs
