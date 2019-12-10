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

#include "cs/path_finding/path_finder.h"

#include <algorithm>
#include <limits>
#include <string>

#include "config_reader/macros.h"

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
