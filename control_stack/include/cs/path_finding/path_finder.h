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
                             const Path2d& proposed_path);

 public:
  explicit PathFinder(const util::Map& map) : map_(map) {}

  virtual Path2d FindPath(const util::Map& dynamic_map,
                          const Eigen::Vector2f& start,
                          const Eigen::Vector2f& goal,
                          ros::Publisher* pub) = 0;
};

}  // namespace path_finding
}  // namespace cs
