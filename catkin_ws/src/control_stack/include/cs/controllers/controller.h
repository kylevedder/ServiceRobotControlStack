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

#include "config_reader/macros.h"
#include "cs/main/debug_pub_wrapper.h"
#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"

namespace cs {
namespace controllers {

enum ControllerType { NAVIGATION, ESCAPE_COLLISION, CONTROLLER_NUM };

static std::string to_string(const ControllerType& t) {
  switch (t) {
    case cs::controllers::ControllerType::NAVIGATION:
      return "NAVIGATION";
    case cs::controllers::ControllerType::ESCAPE_COLLISION:
      return "ESCAPE_COLLISION";
    default:
      NP_CHECK_MSG(false, "Unknown ControllerType " << t);
  }
  return "";
}

class Controller {
 protected:
  cs::main::DebugPubWrapper* dpw_;
  const util::LaserScan& laser_;
  const util::vector_map::VectorMap& map_;
  const state_estimation::StateEstimator& state_estimator_;
  const obstacle_avoidance::ObstacleDetector& obstacle_detector_;
  const motion_planning::PIDController& motion_planner_;

 public:
  Controller() = delete;
  Controller(cs::main::DebugPubWrapper* dpw,
             const util::LaserScan& laser,
             const util::vector_map::VectorMap& map,
             const state_estimation::StateEstimator& state_estimator,
             const obstacle_avoidance::ObstacleDetector& obstacle_detector,
             const motion_planning::PIDController& motion_planner)
      : dpw_(dpw),
        laser_(laser),
        map_(map),
        state_estimator_(state_estimator),
        obstacle_detector_(obstacle_detector),
        motion_planner_(motion_planner) {}
  virtual ~Controller() = default;

  virtual std::pair<ControllerType, util::Twist> Execute() = 0;

  virtual void Reset() = 0;
};

}  // namespace controllers
}  // namespace cs

namespace std {

inline std::ostream& operator<<(std::ostream& os,
                                const cs::controllers::ControllerType& t) {
  os << cs::controllers::to_string(t);
  return os;
}

}  // namespace std