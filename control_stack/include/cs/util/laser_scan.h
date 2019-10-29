#pragma once

#include "sensor_msgs/LaserScan.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "cs/util/constants.h"
#include "cs/util/pose.h"

#include <vector>

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
      const Eigen::Vector2f point(sin(theta) * depth, cos(theta) * depth);
      NP_FINITE(point.x());
      NP_FINITE(point.y());
      robot_frame_points.push_back(point);
    }

    for (auto& point : robot_frame_points) {
      NP_FINITE(point.x());
      NP_FINITE(point.y());
      point = transform * point;
      NP_FINITE(point.x());
      NP_FINITE(point.y());
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
};
}  // namespace util