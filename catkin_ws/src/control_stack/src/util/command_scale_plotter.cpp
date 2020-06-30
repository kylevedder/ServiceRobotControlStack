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

#include "cs/motion_planning/turtlebot_command_scaler.h"

#include "config_reader/config_reader.h"

namespace params {
CONFIG_FLOAT(max_tra_vel, "limits.kMaxTraVel");
CONFIG_FLOAT(max_rot_vel, "limits.kMaxRotVel");
}  // namespace params

int main() {
  const std::string config_file = "src/control_stack/config/nav_config.lua";
  config_reader::ConfigReader reader({config_file});

  cs::motion_planning::TurtlebotCommandScaler s;
  static constexpr int kSamples = 100;

  std::vector<float> onethird;
  std::vector<float> twothird;
  std::vector<float> threethird;

  const float onethirdang = params::CONFIG_max_rot_vel / 3;
  const float twothirdang = 2 * params::CONFIG_max_rot_vel / 3;
  const float threethirdang = params::CONFIG_max_rot_vel;

  for (int i = 0; i < kSamples; ++i) {
    const float tra_speed = params::CONFIG_max_tra_vel / kSamples * i;
    onethird.push_back(s.ScaleCommand({tra_speed, 0, onethirdang}).rot);
    twothird.push_back(s.ScaleCommand({tra_speed, 0, twothirdang}).rot);
    threethird.push_back(s.ScaleCommand({tra_speed, 0, threethirdang}).rot);
  }

  {
    std::ofstream f("onethirdfile.txt");
    f << params::CONFIG_max_tra_vel << ", " << onethirdang << ", ";
    for (const float& v : onethird) {
      f << v << ", ";
    }
  }
  {
    std::ofstream f("twothirdfile.txt");
    f << params::CONFIG_max_tra_vel << ", " << twothirdang << ", ";
    for (const float& v : twothird) {
      f << v << ", ";
    }
  }
  {
    std::ofstream f("threethirdfile.txt");
    f << params::CONFIG_max_tra_vel << ", " << threethirdang << ", ";
    for (const float& v : threethird) {
      f << v << ", ";
    }
  }
}