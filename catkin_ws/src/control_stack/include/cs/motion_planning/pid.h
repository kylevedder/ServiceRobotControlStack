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
#include <utility>
#include <vector>

#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/path_finding/path_finder.h"
#include "cs/state_estimation/state_estimator.h"
#include "cs/util/constants.h"
#include "cs/util/datastructures/circular_buffer.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

namespace cs {
namespace motion_planning {

class PIDController {
 private:
  const util::vector_map::VectorMap& map_;
  const cs::state_estimation::StateEstimator& state_estimator_;

  util::Twist ProposeCommand(const util::Pose& waypoint) const;

  util::Twist ApplyCommandLimits(util::Twist c) const;

  float AlternateCommandCost(
      const TrajectoryRollout& desired_tr,
      const util::Twist& alternate,
      const util::DynamicFeatures& dynamic_features) const;

  std::pair<bool, TrajectoryRollout> IsCommandColliding(
      const util::Twist& commanded_velocity,
      const util::DynamicFeatures& dynamic_features) const;

 public:
  PIDController() = delete;
  PIDController(const util::vector_map::VectorMap& map,
                const cs::state_estimation::StateEstimator& state_estimator)
      : map_(map), state_estimator_(state_estimator) {}

  bool AtPoint(const Eigen::Vector2f& point) const;

  bool AtPose(const util::Pose& pose) const;

  util::Pose EscapeCollisionPose(
      const util::DynamicFeatures& dynamic_features) const;

  util::Twist EscapeCollision(const util::Pose& pose) const;

  util::Twist DriveToPose(const util::DynamicFeatures& dynamic_features,
                          const util::Pose& waypoint) const;
};

}  // namespace motion_planning
}  // namespace cs
