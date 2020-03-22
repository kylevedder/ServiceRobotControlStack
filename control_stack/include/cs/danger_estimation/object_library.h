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

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "cs/util/constants.h"
namespace cs {
namespace danger_estimation {

struct ObjectDescription {
  std::string name;
  float radius;
  float height;

  ObjectDescription() = delete;
  ObjectDescription(const std::string& name,
                    const float& radius,
                    const float& height)
      : name(name), radius(radius), height(height) {}
};

struct Object {
  Eigen::Vector2f position;
  ObjectDescription description;

  Object(const Eigen::Vector2f& position, const ObjectDescription& description)
      : position(position), description(description) {}
};

using ObjectLibrary = std::vector<ObjectDescription>;

ObjectLibrary GetObjectLibrary() {
  return {{"backpack", 0.1, 0.2}, {"person", 0.2, 1.95}, {"fatcan", 0.3, 0.5}};
}

}  // namespace danger_estimation
}  // namespace cs
