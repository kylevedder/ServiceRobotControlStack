#pragma once
// Copyright 2019 kvedder@seas.upenn.edu
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

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

namespace util {
struct Twist {
  Eigen::Vector2f tra;
  float rot;
  Twist() : tra(0, 0), rot(0) {}
  explicit Twist(const Eigen::Vector3f& v) : tra(v.x(), v.y()), rot(v.z()) {}
  explicit Twist(const geometry_msgs::Twist& twist)
      : tra(twist.linear.x, twist.linear.y), rot(twist.angular.z) {}

  Twist(const Eigen::Vector2f& tra, const float& rot) : tra(tra), rot(rot) {}

  Twist(const float x, const float y, const float& rot) : tra(x, y), rot(rot) {}

  bool operator==(const Twist& other) const {
    return (tra == other.tra) && (rot == other.rot);
  }

  bool operator!=(const Twist& other) const { return !(*this == other); }

  Twist operator/(const float& other) const {
    return {tra / other, rot / other};
  }

  void operator/=(const float& other) {
    tra /= other;
    rot /= other;
  }

  Twist operator*(const float& other) const {
    return {tra * other, rot * other};
  }

  void operator*=(const float& other) {
    tra *= other;
    rot *= other;
  }

  util::Twist operator-() const { return {-tra, -rot}; }

  util::Twist operator-(const util::Twist& o) const {
    return {tra - o.tra, rot - o.rot};
  }

  util::Twist operator+(const util::Twist& o) const {
    return {tra + o.tra, rot + o.rot};
  }

  geometry_msgs::Twist ToTwist() const {
    geometry_msgs::Twist twist;
    twist.linear.x = tra.x();
    twist.linear.y = tra.y();
    twist.angular.z = rot;
    return twist;
  }

  Eigen::Affine2f ToAffine2f() const {
    Eigen::Affine2f t(Eigen::Affine2f::Identity());
    t.translate(tra);
    t.rotate(rot);
    return t;
  }

  Eigen::Affine3f ToAffine3f() const {
    Eigen::Affine3f t(Eigen::Affine3f::Identity());
    t.translate(Eigen::Vector3f(tra.x(), tra.y(), 0));
    t.rotate(Eigen::AngleAxisf(rot, Eigen::Vector3f::UnitZ()));
    return t;
  }

  bool IsFinite() const {
    return std::isfinite(tra.x()) && std::isfinite(tra.y()) &&
           std::isfinite(rot);
  }

  friend std::ostream& operator<<(std::ostream& os, const Twist& obj);
};

inline std::ostream& operator<<(std::ostream& os, const Twist& obj) {
  os << "(" << obj.tra.x() << ", " << obj.tra.y() << "), " << obj.rot;
  return os;
}
}  // namespace util
