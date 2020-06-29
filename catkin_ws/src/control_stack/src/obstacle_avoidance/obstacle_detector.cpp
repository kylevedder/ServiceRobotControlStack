// Copyright 2019 - 2020 kvedder@seas.upenn.edu
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
#include "cs/obstacle_avoidance/obstacle_detector.h"

#include <visualization_msgs/MarkerArray.h>

#include <limits>
#include <utility>
#include <vector>

#include "config_reader/macros.h"
#include "cs/motion_planning/trajectory_rollout.h"
#include "cs/util/laser_scan.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"

namespace cs {
namespace obstacle_avoidance {

namespace od_params {
CONFIG_FLOAT(kMinDistanceThreshold, "od.kMinDistanceThreshold");
CONFIG_FLOAT(kDistanceFromMax, "od.kDistanceFromMax");
CONFIG_FLOAT(kProposedTranslationStdDev, "od.kProposedTranslationStdDev");
CONFIG_FLOAT(kProposedRotationStdDev, "od.kProposedRotationStdDev");

CONFIG_FLOAT(max_dist_between_readings,
             "od.clustering.max_dist_between_readings");
CONFIG_FLOAT(min_distance_btw_readings_to_reason_angle,
             "od.clustering.min_distance_btw_readings_to_reason_angle");
CONFIG_FLOAT(line_similarity, "od.clustering.line_similarity");

CONFIG_STRING(map_tf_frame, "frames.map_tf_frame");

CONFIG_FLOAT(is_wall_threshold, "od.is_wall_threshold");
}  // namespace od_params

ObstacleDetector::ObstacleDetector(const util::vector_map::VectorMap& map)
    : map_(map), random_gen_(0) {}

util::DynamicFeatures ObstacleDetector::GetNonMapPoints(
    const util::Pose& observation_pose,
    const util::LaserScan& observation) const {
  return util::DynamicFeatures(observation.TransformPointsFrameSparse(
      observation_pose.ToAffine(), [&observation](const float& f) {
        return (f <= observation.ros_laser_scan_.range_max) &&
               (f >= observation.ros_laser_scan_.range_min);
      }));
}

void ObstacleDetector::UpdateObservation(const util::Pose& observation_pose,
                                         const util::LaserScan& observation) {
  UpdateObservation(observation_pose, observation, nullptr);
}

void ObstacleDetector::UpdateObservation(const util::Pose& observation_pose,
                                         const util::LaserScan& observation,
                                         ros::Publisher* pub) {
  dynamic_features_ = GetNonMapPoints(observation_pose, observation);

  static constexpr bool kDebug = true;
  if (kDebug && (pub != nullptr)) {
    static visualization_msgs::MarkerArray old_marker;
    visualization_msgs::MarkerArray new_marker;
    visualization::PointsToSpheres(dynamic_features_.features,
                                   od_params::CONFIG_map_tf_frame,
                                   "non_map_pts",
                                   &new_marker);
    for (auto& m : old_marker.markers) {
      m.action = m.DELETE;
    }
    pub->publish(old_marker);
    pub->publish(new_marker);
    old_marker = new_marker;
  }
}

const util::DynamicFeatures& ObstacleDetector::GetDynamicFeatures() const {
  return dynamic_features_;
}

}  // namespace obstacle_avoidance
}  // namespace cs
