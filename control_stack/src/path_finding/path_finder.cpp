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

namespace cs {
namespace path_finding {

namespace params {
CONFIG_FLOAT(switch_historesis_threshold, "switch_historesis_threshold");
};

Path2d PathFinder::UsePrevPathOrUpdate(const util::Map& dynamic_map,
                                       const Path2d& proposed_path) {
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
      (params::CONFIG_switch_historesis_threshold + proposed_path.cost <
           prev_path_.cost ||
       distance_between_starts >
           math_util::Sq(params::CONFIG_switch_historesis_threshold))) {
    ROS_INFO("Proposed path better");
    prev_path_ = proposed_path;
    return proposed_path;
  }
  return prev_path_;
}
}  // namespace path_finding
}  // namespace cs
