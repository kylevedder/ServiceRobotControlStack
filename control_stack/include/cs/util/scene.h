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

#include <eigen3/Eigen/Core>

#include <vector>

struct VertPlane {
  Eigen::Vector3f position;
  float y;
  float z;

  VertPlane() : position(Eigen::Vector3f::Zero()), y(0), z(0) {}
  VertPlane(const Eigen::Vector3f& position, const float& y, const float& z)
      : position(position), y(y), z(z) {}

  float DistanceFromPlane(const Eigen::Vector3f& query) {
    Eigen::Vector3f diff = (position - query);
    const float y_diff = fabs(diff.y()) - (y / 2);
    const float z_diff = fabs(diff.z()) - (z / 2);
    if (y_diff < 0 && z_diff < 0) {
      return fabs(diff.x());
    }

    if (y_diff >= 0) {
      diff.y() = y_diff;
    } else {
      diff.y() = 0;
    }
    if (z_diff >= 0) {
      diff.z() = z_diff;
    } else {
      diff.z() = 0;
    }
    return diff.norm();
  }
};

struct HorizPlane {
  Eigen::Vector3f position;
  float x;
  float y;

  HorizPlane() : position(Eigen::Vector3f::Zero()), x(0), y(0) {}
  HorizPlane(const Eigen::Vector3f& position, const float& x, const float& y)
      : position(position), x(x), y(y) {}

  float DistanceFromPlane(const Eigen::Vector3f& query) {
    Eigen::Vector3f diff = (position - query);
    const float x_diff = fabs(diff.x()) - (x / 2);
    const float y_diff = fabs(diff.y()) - (y / 2);
    if (x_diff < 0 && y_diff < 0) {
      return fabs(diff.z());
    }

    if (x_diff >= 0) {
      diff.x() = x_diff;
    } else {
      diff.x() = 0;
    }
    if (y_diff >= 0) {
      diff.y() = y_diff;
    } else {
      diff.y() = 0;
    }
    return diff.norm();
  }
};

struct Scene {
  std::vector<VertPlane> vertical_planes;
  std::vector<HorizPlane> horizontal_planes;
};

visualization_msgs::MarkerArray MakeSceneGroundTruth() {
  visualization_msgs::MarkerArray arr;
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time();
    marker.ns = "wall";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 3;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.0001;
    marker.scale.y = 10;
    marker.scale.z = 10;
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    arr.markers.push_back(marker);
  }
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time();
    marker.ns = "floor";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -1;
    marker.scale.x = 10;
    marker.scale.y = 10;
    marker.scale.z = 0.0001;
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    arr.markers.push_back(marker);
  }
  return arr;
}
