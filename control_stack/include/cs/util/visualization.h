#pragma once

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"

namespace visualization {
void DrawPose(const util::Pose& pose, const std::string& frame_id,
              const std::string& ns, const float r, const float g,
              const float b, const float alpha,
              visualization_msgs::MarkerArray* arr, const float z = 0.0f) {
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = arr->markers.size();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.tra.x();
    marker.pose.position.y = pose.tra.y();
    marker.pose.position.z = z;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = alpha;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    arr->markers.push_back(marker);
  }
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = arr->markers.size();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point start;
    start.x = pose.tra.x();
    start.y = pose.tra.y();
    start.z = z;
    geometry_msgs::Point end;
    static constexpr float kArrowLength = 0.1f;
    const Eigen::Vector2f delta(math_util::Cos(pose.rot) * kArrowLength,
                                math_util::Sin(pose.rot) * kArrowLength);
    end.x = pose.tra.x() + delta.x();
    end.y = pose.tra.y() + delta.y();
    end.z = z;
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.scale.x = 0.01;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = alpha;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    arr->markers.push_back(marker);
  }
}

visualization_msgs::Marker ToLineList(
    const util::LaserScan& laser, const util::Pose& robot_pose,
    const util::Map& map, const std::string& frame_id, const std::string& ns,
    const float r, const float g, const float b, const float alpha) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  const float& min_angle = laser.ros_laser_scan_.angle_min;
  const float& angle_delta = laser.ros_laser_scan_.angle_increment;
  const float& range_min = laser.ros_laser_scan_.range_min;
  const float& range_max = laser.ros_laser_scan_.range_max;

  for (size_t i = 0; i < laser.ros_laser_scan_.ranges.size(); ++i) {
    const float angle = math_util::AngleMod(
        min_angle + angle_delta * static_cast<float>(i) + robot_pose.rot);
    const util::Pose ray(robot_pose.tra, angle);
    const float dist =
        map.MinDistanceAlongRay(ray, range_min, range_max - kEpsilon);
    const Eigen::Vector2f& s = robot_pose.tra;
    const Eigen::Vector2f e =
        Eigen::Rotation2Df(ray.rot) * Eigen::Vector2f(dist, 0) + s;

    geometry_msgs::Point start;
    start.x = s.x();
    start.y = s.y();
    geometry_msgs::Point end;
    end.x = e.x();
    end.y = e.y();

    marker.points.push_back(start);
    marker.points.push_back(end);
  }
  marker.scale.x = 0.01;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = alpha;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  return marker;
}

}  // namespace visualization