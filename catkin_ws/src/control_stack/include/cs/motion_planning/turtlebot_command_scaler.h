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

#include "config_reader/config_reader.h"
#include "cs/motion_planning/command_scaler.h"

namespace cs {
namespace motion_planning {

namespace params {
CONFIG_FLOAT(rotation_zero_threshold, "cmd_scaler.rotation_zero_threshold");
CONFIG_FLOAT(rotation_min_effect_threshold,
             "cmd_scaler.rotation_min_effect_threshold");
CONFIG_FLOAT(rotation_translation_scaler,
             "cmd_scaler.rotation_translation_scaler");
}  // namespace params

class TurtlebotCommandScaler : public CommandScaler {
 public:
  TurtlebotCommandScaler() : CommandScaler(){};
  ~TurtlebotCommandScaler() = default;

  util::Twist ScaleCommand(util::Twist cmd) const {
    const float arot = std::abs(cmd.rot);
    if (arot < params::CONFIG_rotation_zero_threshold) {
      cmd.rot = 0;
    } else if (arot <= params::CONFIG_rotation_min_effect_threshold) {
      cmd.rot = params::CONFIG_rotation_min_effect_threshold *
                math_util::Sign(cmd.rot);
    }
    cmd.rot *= (1 + math_util::Sq(std::abs(cmd.tra.x()) *
                                  params::CONFIG_rotation_translation_scaler));
    return cmd;
  };
};

}  // namespace motion_planning
}  // namespace cs
