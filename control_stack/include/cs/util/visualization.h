#pragma once
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <tuple>
#include <vector>

#include "cs/obstacle_avoidance/trajectory_rollout.h"
#include "cs/path_finding/path_finder.h"
#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/plane_fit.h"
#include "cs/util/pose.h"

namespace visualization {

inline visualization_msgs::Marker DrawPlane(
    const util::Plane& p,
    const std::string& frame_id,
    const std::string& ns,
    const Eigen::Quaternionf& quaternion) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  const Eigen::Vector3f center = p.anchor + p.v1 / 2 + p.v2 / 2;
  marker.pose.position.x = center.x();
  marker.pose.position.y = center.y();
  marker.pose.position.z = center.z();
  marker.pose.orientation.w = quaternion.w();
  marker.pose.orientation.x = quaternion.x();
  marker.pose.orientation.y = quaternion.y();
  marker.pose.orientation.z = quaternion.z();
  marker.scale.x = p.v1.norm();
  marker.scale.y = p.v2.norm();
  marker.scale.z = 0.0001;
  marker.color.a = 1;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  return marker;
}

inline visualization_msgs::Marker DrawWalls(
    const std::vector<util::Wall>& walls,
    const std::string& frame_id,
    const std::string& ns) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  for (const auto& w : walls) {
    geometry_msgs::Point p1;
    p1.x = w.p1.x();
    p1.y = w.p1.y();
    geometry_msgs::Point p2;
    p2.x = w.p2.x();
    p2.y = w.p2.y();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  marker.scale.x = 0.02;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  return marker;
}

inline visualization_msgs::Marker DrawPath(const cs::path_finding::Path2d& path,
                                           const std::string& frame_id,
                                           const std::string& ns) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.02;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;

  if (!path.IsValid() || path.waypoints.size() < 2) {
    return marker;
  }

  for (size_t i = 0; i < path.waypoints.size() - 1; ++i) {
    geometry_msgs::Point p1;
    p1.x = path.waypoints[i].x();
    p1.y = path.waypoints[i].y();
    geometry_msgs::Point p2;
    p2.x = path.waypoints[i + 1].x();
    p2.y = path.waypoints[i + 1].y();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  return marker;
}

inline void DrawPoints(const std::vector<Eigen::Vector2f>& points,
                       const std::string& frame_id,
                       const std::string& ns,
                       const float r,
                       const float g,
                       const float b,
                       const float alpha,
                       visualization_msgs::MarkerArray* arr,
                       const float z = 0.0f) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = arr->markers.size();
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  int i = 0;
  for (const auto& p : points) {
    if (++i % 3 != 0) {
      continue;
    }
    geometry_msgs::Point pm;
    pm.x = p.x();
    pm.y = p.y();
    pm.z = z;
    marker.points.push_back(pm);
  }
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = alpha;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  arr->markers.push_back(marker);
}

inline void DrawPose(const util::Pose& pose,
                     const std::string& frame_id,
                     const std::string& ns,
                     const float r,
                     const float g,
                     const float b,
                     const float alpha,
                     visualization_msgs::MarkerArray* arr,
                     const float z = 0.0f) {
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
    const Eigen::Vector2f delta(std::cos(pose.rot) * kArrowLength,
                                std::sin(pose.rot) * kArrowLength);
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

inline void DrawTrajectoryRollout(
    const cs::obstacle_avoidance::TrajectoryRollout& tr,
    const std::string& frame_id,
    const std::string& ns,
    visualization_msgs::MarkerArray* arr,
    const bool is_colliding = false) {
  DrawPose({tr.rotate_circle_center, 0},
           frame_id,
           ns + "rotate_center",
           0,
           0,
           1,
           1,
           arr);
  if (!is_colliding) {
    DrawPose(tr.start_pose, frame_id, ns + "start_pose", 0, 1, 0, 1, arr);
    DrawPose(
        tr.achieved_vel_pose, frame_id, ns + "achieved_pose", 0, 1, 0, 1, arr);
    DrawPose(tr.final_pose, frame_id, ns + "achieved_pose", 0, 1, 0, 1, arr);
  } else {
    DrawPose(tr.start_pose, frame_id, ns + "start_pose", 1, 0, 0, 1, arr);
    DrawPose(
        tr.achieved_vel_pose, frame_id, ns + "achieved_pose", 1, 0, 0, 1, arr);
    DrawPose(tr.final_pose, frame_id, ns + "achieved_pose", 1, 0, 0, 1, arr);
  }
}

inline std::tuple<float, float, float> IndexToDistinctRBG(const size_t idx) {
  static const std::vector<std::tuple<float, float, float>> color_lst(
      {std::make_tuple(0, 0, 0),
       std::make_tuple(1, 0, 0),
       std::make_tuple(0, 1, 0),
       std::make_tuple(0, 0, 1),
       std::make_tuple(1, 1, 0),
       std::make_tuple(0, 1, 1),
       std::make_tuple(1, 0, 1),
       std::make_tuple(1, 1, 1),
       std::make_tuple(0.5, 0, 0)});

  if (idx < color_lst.size()) {
    return color_lst[idx];
  }

  const int val = (idx + 500) * 5000;
  const float r = static_cast<float>(val % 255) / 255;
  const float g = static_cast<float>(val % (255 * 255)) / 255;
  const float b = static_cast<float>(val % (255 * 255 * 255)) / 255;
  return std::make_tuple(r, g, b);
}

inline void PointsToSpheres(const std::vector<Eigen::Vector2f>& points,
                            const std::string& frame_id,
                            const std::string& ns,
                            visualization_msgs::MarkerArray* arr,
                            const float r = 1,
                            const float g = 0,
                            const float b = 0,
                            const float z = 0,
                            const float radius = 0.1) {
  for (const auto& v : points) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = arr->markers.size();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = v.x();
    marker.pose.position.y = v.y();
    marker.pose.position.z = z;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.a = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    arr->markers.push_back(marker);
  }
}

inline visualization_msgs::Marker ToLine(const Eigen::Vector2f& p1,
                                         const Eigen::Vector2f& p2,
                                         const std::string& frame_id,
                                         const std::string& ns,
                                         const int id = 0,
                                         const float r = 1,
                                         const float g = 0,
                                         const float b = 0,
                                         const float width = 0.1) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = width;
  marker.color.a = 1;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;

  geometry_msgs::Point start;
  start.x = p1.x();
  start.y = p1.y();
  geometry_msgs::Point end;
  end.x = p2.x();
  end.y = p2.y();

  marker.points.push_back(start);
  marker.points.push_back(end);

  return marker;
}

inline visualization_msgs::Marker PointsToLineList(
    const std::vector<Eigen::Vector2f>& points,
    const util::Pose& robot_pose,
    const std::string& frame_id,
    const std::string& ns,
    const float r = 1,
    const float g = 0,
    const float b = 0) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;

  for (const Eigen::Vector2f& v : points) {
    geometry_msgs::Point start;
    start.x = robot_pose.tra.x();
    start.y = robot_pose.tra.y();
    geometry_msgs::Point end;
    end.x = v.x();
    end.y = v.y();

    marker.points.push_back(start);
    marker.points.push_back(end);
  }

  return marker;
}

inline visualization_msgs::Marker LaserToLineList(const util::LaserScan& laser,
                                                  const util::Pose& robot_pose,
                                                  const util::Map& map,
                                                  const std::string& frame_id,
                                                  const std::string& ns,
                                                  const float r,
                                                  const float g,
                                                  const float b,
                                                  const float alpha) {
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

inline visualization_msgs::Marker MakeCylinder(const Eigen::Vector2f& position,
                                               const float radius,
                                               const float height,
                                               const std::string& frame_id,
                                               const std::string& ns,
                                               const float r,
                                               const float g,
                                               const float b,
                                               const float alpha) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 2 * radius;
  marker.scale.y = 2 * radius;
  marker.scale.z = height;
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.color.a = alpha;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  return marker;
}

}  // namespace visualization
