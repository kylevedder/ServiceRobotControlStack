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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <string>
#include <vector>

#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "shared/math/math_util.h"

namespace util {
namespace physics {

util::Twist ApplyCommandLimits(util::Twist commanded_velocity,
                               const float time_delta,
                               const util::Twist& current_velocity,
                               const float max_tra_velocity,
                               const float max_tra_acceleration,
                               const float max_rot_velocity,
                               const float max_rot_acceleration);

struct CommandDeltaCurve {
  util::Pose current_position_wf;
  util::Twist end_velocity_rf;
  util::Pose end_position_wf;
  float rotate_circle_radius;
  Eigen::Vector2f rotate_circle_center_wf;

  CommandDeltaCurve()
      : current_position_wf(),
        end_velocity_rf(),
        end_position_wf(),
        rotate_circle_radius(0),
        rotate_circle_center_wf(0, 0) {}
};

struct CommandDeltaStraight {
  util::Pose current_position_wf;
  util::Twist end_velocity_rf;
  util::Pose end_position_wf;
  CommandDeltaStraight()
      : current_position_wf(), end_velocity_rf(), end_position_wf(){};
};

struct CommandDelta {
  enum class Type { CURVE, STRAIGHT };
  Type type;
  CommandDeltaStraight straight;
  CommandDeltaCurve curve;

  CommandDelta() = delete;
  explicit CommandDelta(const CommandDeltaCurve& curve)
      : type(Type::CURVE), straight(), curve(curve) {}
  explicit CommandDelta(const CommandDeltaStraight& straight)
      : type(Type::STRAIGHT), straight(straight), curve() {}

  const util::Pose& GetEndPosition() const {
    switch (type) {
      case Type::CURVE:
        return curve.end_position_wf;
      case Type::STRAIGHT:
        return straight.end_position_wf;
    }
  }

  const util::Twist& GetEndVelocity() const {
    switch (type) {
      case Type::CURVE:
        return curve.end_velocity_rf;
      case Type::STRAIGHT:
        return straight.end_velocity_rf;
    }
  }
};

struct StopDelta {
  util::Pose current_position_wf;
  util::Pose stop_position_wf;
};

CommandDelta ComputeCommandDelta(const util::Pose& current_position_wf,
                                 const util::Twist& current_velocity_rf,
                                 const util::Twist& commanded_velocity_rf,
                                 const float time_delta);

StopDelta ComputeFullStop(const CommandDelta& command_delta,
                          const float max_tra_acceleration);

}  // namespace physics
}  // namespace util

namespace std {
inline std::ostream& operator<<(std::ostream& os,
                                const util::physics::CommandDelta::Type& t) {
  switch (t) {
    case util::physics::CommandDelta::Type::STRAIGHT:
      os << "STRAIGHT";
      return os;
    case util::physics::CommandDelta::Type::CURVE:
      os << "CURVE";
      return os;
  }
}
}  // namespace std