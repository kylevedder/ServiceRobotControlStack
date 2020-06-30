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

#include "cs/controllers/controller.h"

namespace cs {
namespace controllers {

struct EscapeCollisionWaypoint {
  bool initialized;
  Eigen::Vector2f waypoint;
  Eigen::Vector2f colliding_point;

  EscapeCollisionWaypoint()
      : initialized(false),
        waypoint(Eigen::Vector2f::Zero()),
        colliding_point(Eigen::Vector2f::Zero()) {}

  EscapeCollisionWaypoint(bool initialized,
                          const Eigen::Vector2f& waypoint,
                          const Eigen::Vector2f& colliding_point)
      : initialized(initialized),
        waypoint(waypoint),
        colliding_point(colliding_point) {}
};

class EscapeCollisionController : public Controller {
  EscapeCollisionWaypoint escape_waypoint_;
  visualization_msgs::Marker colliding_marker_;

 public:
  EscapeCollisionController() = delete;
  EscapeCollisionController(
      cs::main::DebugPubWrapper* dpw,
      const util::LaserScan& laser,
      const util::vector_map::VectorMap& map,
      const state_estimation::StateEstimator& state_estimator,
      const obstacle_avoidance::ObstacleDetector& obstacle_detector,
      const motion_planning::PIDController& motion_planner);
  ~EscapeCollisionController() = default;

  std::pair<ControllerType, util::Twist> Execute() override;

  void Reset() override;
};

}  // namespace controllers
}  // namespace cs