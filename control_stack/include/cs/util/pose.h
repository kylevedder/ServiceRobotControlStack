#pragma once

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
  Pose() : tra(), rot(){};
  explicit Pose(const geometry_msgs::Twist& twist)
      : tra(twist.linear.x, twist.linear.y), rot(twist.angular.z) {}
  explicit Pose(const geometry_msgs::Pose& pose)
      : tra(pose.position.x, pose.position.y), rot(0) {
    const auto& q = pose.orientation;
    const float siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    rot = atan2(siny_cosp, cosy_cosp);
  }
  Pose(const Eigen::Vector2f& tra, const float& rot) : tra(tra), rot(rot){};

  bool operator==(const Pose& other) const {
    return (tra == other.tra) && (rot == other.rot);
  };

  bool operator!=(const Pose& other) const { return !(*this == other); }

  Pose operator/(const float& other) const {
    return {tra / other, rot / other};
  }

  void operator/=(const float& other) {
    tra /= other;
    rot /= other;
  }

  Pose operator*(const float& other) const {
    return {tra * other, rot * other};
  }

  void operator*=(const float& other) {
    tra *= other;
    rot *= other;
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
    Eigen::Affine2f transform(Eigen::Affine2f::Identity());
    transform.translate(tra);
    transform.rotate(rot);
    return transform;
  }
};
}  // namespace util