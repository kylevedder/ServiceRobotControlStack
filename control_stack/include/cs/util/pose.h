#pragma once
// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "cs/util/math_util.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

namespace util {
struct Pose {
  Eigen::Vector2f tra;
  float rot;
  Pose() : tra(), rot() {}
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

  bool IsFinite() const {
    return std::isfinite(tra.x()) && std::isfinite(tra.y()) &&
           std::isfinite(rot);
  }
};
}  // namespace util
