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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <tuple>
#include <vector>

#include "cs/obstacle_avoidance/trajectory_rollout.h"
#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"

namespace visualization {

visualization_msgs::Marker DrawWalls(const std::vector<util::Wall>& walls,
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

void DrawTrajectoryRollout(const cs::obstacle_avoidance::TrajectoryRollout& tr,
                           const std::string& frame_id, const std::string& ns,
                           visualization_msgs::MarkerArray* arr,
                           const bool is_colliding = false) {
  DrawPose({tr.rotate_circle_center, 0}, frame_id, ns + "rotate_center", 0, 0,
           1, 1, arr);
  if (!is_colliding) {
    DrawPose(tr.start_pose, frame_id, ns + "start_pose", 0, 1, 0, 1, arr);
    DrawPose(tr.achieved_vel_pose, frame_id, ns + "achieved_pose", 0, 1, 0, 1,
             arr);
    DrawPose(tr.final_pose, frame_id, ns + "achieved_pose", 0, 1, 0, 1, arr);
  } else {
    DrawPose(tr.start_pose, frame_id, ns + "start_pose", 1, 0, 0, 1, arr);
    DrawPose(tr.achieved_vel_pose, frame_id, ns + "achieved_pose", 1, 0, 0, 1,
             arr);
    DrawPose(tr.final_pose, frame_id, ns + "achieved_pose", 1, 0, 0, 1, arr);
  }
}

std::tuple<float, float, float> IndexToDistinctRBG(const size_t idx) {
  static const std::vector<std::tuple<float, float, float>> color_lst(
      {std::make_tuple(0, 0, 0), std::make_tuple(1, 0, 0),
       std::make_tuple(0, 1, 0), std::make_tuple(0, 0, 1),
       std::make_tuple(1, 1, 0), std::make_tuple(0, 1, 1),
       std::make_tuple(1, 0, 1), std::make_tuple(1, 1, 1),
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

void PointsToSpheres(const std::vector<Eigen::Vector2f>& points,
                     const std::string& frame_id, const std::string& ns,
                     visualization_msgs::MarkerArray* arr, const float r = 1,
                     const float g = 0, const float b = 0, const float z = 0) {
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
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    arr->markers.push_back(marker);
  }
}

visualization_msgs::Marker ToLine(const Eigen::Vector2f& p1,
                                  const Eigen::Vector2f& p2,
                                  const std::string& frame_id,
                                  const std::string& ns, const int id = 0,
                                  const float r = 1, const float g = 0,
                                  const float b = 0) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
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

visualization_msgs::Marker PointsToLineList(
    const std::vector<Eigen::Vector2f>& points, const util::Pose& robot_pose,
    const std::string& frame_id, const std::string& ns, const float r = 1,
    const float g = 0, const float b = 0) {
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

visualization_msgs::Marker LaserToLineList(
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

visualization_msgs::Marker MakeCylinder(const Eigen::Vector2f& position,
                                        const float radius, const float height,
                                        const std::string& frame_id,
                                        const std::string& ns, const float r,
                                        const float g, const float b,
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
