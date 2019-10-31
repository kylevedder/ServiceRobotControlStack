#pragma once

#include <ros/ros.h>
#include <vector>

#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"

namespace cs {
namespace obstacle_avoidance {

class ObstacleDetector {
 public:
  ObstacleDetector() = delete;
  explicit ObstacleDetector(util::Map const& map);

  void UpdateObservation(const util::Pose& observation_pose,
                         const util::LaserScan& observation,
                         ros::Publisher* pub);

  void DrawDynamic(ros::Publisher* pub) const;

  const std::vector<util::Wall>& GetDynamicWalls() const;

 private:
  std::vector<Eigen::Vector2f> GetNonMapPoints(
      const util::Pose& observation_pose,
      const util::LaserScan& observation) const;

  util::Map map_;
  std::vector<util::Wall> dynamic_walls_;
};

}  // namespace obstacle_avoidance
}  // namespace cs