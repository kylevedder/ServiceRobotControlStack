#pragma once
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

#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>
#include "cs/util/pose.h"
#include "shared/math/line2d.h"

namespace util {

using Wall = geometry::Line2f;

struct Map {
  std::vector<Wall> walls;

  Map() = default;
  explicit Map(const std::string& filepath);
  explicit Map(std::vector<Wall> walls);

  float MinDistanceAlongRay(const util::Pose& ray,
                            const float& min_depth,
                            const float& max_depth) const;

  float MinDistanceToWall(const Eigen::Vector2f& observation) const;

  visualization_msgs::Marker ToMarker() const;

  Map Merge(const Map& other) const;
};

}  // namespace util
