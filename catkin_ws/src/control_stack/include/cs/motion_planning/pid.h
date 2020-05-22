#pragma once
// Copyright 2020 kvedder@seas.upenn.edu
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

#include <algorithm>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "cs/path_finding/path_finder.h"
#include "cs/state_estimation/state_estimation.h"
#include "cs/util/constants.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

namespace cs {
namespace motion_planning {

class PIDController {
 private:
  const util::Map& map_;
  const cs::state_estimation::StateEstimator& state_estimator_;
  util::Map complete_map_;
  util::Pose est_world_pose_;
  util::Twist est_velocity_;

  util::Twist ProposeCommand(const Eigen::Vector2f& waypoint) const;

  util::Twist ApplyCommandLimits(util::Twist c) const;

  float AlternateCommandCost(const util::Twist& desired,
                             const util::Twist& alternate) const;

  bool IsCommandColliding(const util::Twist& commanded_velocity) const;

 public:
  PIDController(const util::Map& map,
                const cs::state_estimation::StateEstimator& state_estimator)
      : map_(map),
        state_estimator_(state_estimator),
        complete_map_(map),
        est_world_pose_(),
        est_velocity_() {}

  util::Twist DriveToPoint(const util::Map& dynamic_map,
                           const Eigen::Vector2f& waypoint);
};

}  // namespace motion_planning
}  // namespace cs
