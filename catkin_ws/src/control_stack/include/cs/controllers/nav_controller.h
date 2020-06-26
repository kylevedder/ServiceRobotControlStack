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

class NavController : public Controller {
  path_finding::GlobalPathFinder<path_finding::AStar<3, 20000, false>>
      global_path_finder_;
  path_finding::AStar<5, 500, false> local_path_finder_;
  util::Pose current_goal_;
  size_t current_goal_index_;

 public:
  NavController() = delete;
  NavController(cs::main::DebugPubWrapper* dpw,
                const util::LaserScan& laser,
                const util::vector_map::VectorMap& map,
                const state_estimation::StateEstimator& state_estimator,
                const obstacle_avoidance::ObstacleDetector& obstacle_detector,
                const motion_planning::PIDController& motion_planner);
  ~NavController() = default;

  std::pair<ControllerType, util::Twist> Execute() override;

  void Reset() override;
};

}  // namespace controllers
}  // namespace cs