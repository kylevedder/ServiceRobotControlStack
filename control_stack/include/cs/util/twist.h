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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "cs/util/math_util.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

namespace util {
struct Twist {
  Eigen::Vector2f tra;
  float rot;
  Twist() : tra(0, 0), rot(0) {}
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
};
}  // namespace util
