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

#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>

#include "cs/util/constants.h"
#include "cs/util/pose.h"

namespace util {
class LaserScan {
 public:
  sensor_msgs::LaserScan ros_laser_scan_;

  LaserScan() = default;
  explicit LaserScan(const sensor_msgs::LaserScan& ros_laser_scan)
      : ros_laser_scan_(ros_laser_scan) {}

  bool IsEmpty() const { return ros_laser_scan_.ranges.empty(); }

  std::vector<Eigen::Vector2f> TransformPointsFrameSparse(
      const Eigen::Affine2f& transform) const {
    std::vector<Eigen::Vector2f> robot_frame_points;
    for (size_t i = 0; i < ros_laser_scan_.ranges.size(); ++i) {
      const float& depth = ros_laser_scan_.ranges[i];
      if (!std::isfinite(depth)) {
        continue;
      }

      const float theta =
          ros_laser_scan_.angle_min + i * ros_laser_scan_.angle_increment;
      const Eigen::Vector2f point(cos(theta) * depth, sin(theta) * depth);
      NP_FINITE(point.x());
      NP_FINITE(point.y());
      const Eigen::Vector2f transformed_point = transform * point;
      NP_FINITE(transformed_point.x());
      NP_FINITE(transformed_point.y());
      robot_frame_points.push_back(transformed_point);
    }

    return robot_frame_points;
  }

  Eigen::Vector2f GetRayEndpoint(const size_t ray_index,
                                 const util::Pose& obs_pose) const {
    NP_CHECK(ray_index < ros_laser_scan_.ranges.size());
    const float ray_angle_obs_frame = math_util::AngleMod(
        ros_laser_scan_.angle_min +
        ros_laser_scan_.angle_increment * ray_index + obs_pose.rot);
    const float& range_max = ros_laser_scan_.range_max;

    const Eigen::Vector2f endpoint =
        Eigen::Vector2f(math_util::Cos(ray_angle_obs_frame) * range_max,
                        math_util::Sin(ray_angle_obs_frame) * range_max) +
        obs_pose.tra;
    NP_CHECK((endpoint - obs_pose.tra).norm() <= range_max + kEpsilon);
    return endpoint;
  }

  struct LaserScanIterator {
    const util::LaserScan* laser_scan_;
    int index_;
    LaserScanIterator() = delete;
    LaserScanIterator(const util::LaserScan* laser_scan, int index)
        : laser_scan_(laser_scan), index_(index) {}

    bool operator==(const LaserScanIterator& other) const {
      return (other.laser_scan_ == laser_scan_) && (other.index_ == index_);
    }

    bool operator!=(const LaserScanIterator& other) const {
      return !(*this == other);
    }

    void operator++() { index_++; }

    float Depth() const { return laser_scan_->ros_laser_scan_.ranges[index_]; }

    float Angle() const {
      return laser_scan_->ros_laser_scan_.angle_min +
             index_ * laser_scan_->ros_laser_scan_.angle_increment;
    }

    struct DepthAngle {
      float depth;
      float angle;
      DepthAngle() = delete;
      DepthAngle(float depth, float angle) : depth(depth), angle(angle) {}
    };

    DepthAngle operator*() const { return {Depth(), Angle()}; }
  };

  LaserScanIterator begin() { return LaserScanIterator(this, 0); }

  LaserScanIterator begin() const { return LaserScanIterator(this, 0); }

  LaserScanIterator end() {
    return LaserScanIterator(this, ros_laser_scan_.ranges.size());
  }

  LaserScanIterator end() const {
    return LaserScanIterator(this, ros_laser_scan_.ranges.size());
  }
};
}  // namespace util
