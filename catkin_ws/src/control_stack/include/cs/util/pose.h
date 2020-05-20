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
#include "shared/math/math_util.h"

namespace util {
struct Pose {
  Eigen::Vector2f tra;
  float rot;
  Pose() : tra(0, 0), rot(0) {}
  explicit Pose(const geometry_msgs::Twist& twist)
      : tra(twist.linear.x, twist.linear.y), rot(twist.angular.z) {}
  explicit Pose(const geometry_msgs::Pose& pose)
      : tra(pose.position.x, pose.position.y), rot(0) {
    const auto& q = pose.orientation;
    const float siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    rot = atan2(siny_cosp, cosy_cosp);
  }
  Pose(const Eigen::Vector2f& tra, const float& rot) : tra(tra), rot(rot) {}

  Pose(const float x, const float y, const float& rot) : tra(x, y), rot(rot) {}

  bool operator==(const Pose& other) const {
    return (tra == other.tra) && (rot == other.rot);
  }

  bool operator!=(const Pose& other) const { return !(*this == other); }

  Pose operator/(const float& other) const {
    return {tra / other, math_util::AngleMod(rot / other)};
  }

  void operator/=(const float& other) {
    tra /= other;
    rot /= other;
    rot = math_util::AngleMod(rot);
  }

  Pose operator*(const float& other) const {
    return {tra * other, math_util::AngleMod(rot * other)};
  }

  void operator*=(const float& other) {
    tra *= other;
    rot *= other;
    rot = math_util::AngleMod(rot);
  }

  util::Pose operator-() const { return {-tra, -rot}; }

  util::Pose operator-(const util::Pose& o) const {
    return {tra - o.tra, math_util::AngleMod(rot - o.rot)};
  }

  util::Pose operator+(const util::Pose& o) const {
    return {tra + o.tra, math_util::AngleMod(rot + o.rot)};
  }

  geometry_msgs::Twist ToTwist() const {
    geometry_msgs::Twist twist;
    twist.linear.x = tra.x();
    twist.linear.y = tra.y();
    twist.angular.z = rot;
    return twist;
  }

  Eigen::Affine2f ToAffine() const {
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

  friend std::ostream& operator<<(std::ostream& os, const Pose& obj);
};

inline std::ostream& operator<<(std::ostream& os, const Pose& obj) {
  os << "(" << obj.tra.x() << ", " << obj.tra.y() << "), " << obj.rot;
  return os;
}
}  // namespace util
