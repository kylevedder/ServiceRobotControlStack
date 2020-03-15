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

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <fstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "cs/util/point_cloud.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

using PC16 = util::pc::PointCloud<util::pc::Point16>;

std::tuple<std::string> GetArgs(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: [base file]" << std::endl;
    exit(0);
  }
  return {argv[1]};
}

PC16 GetPC(rosbag::Bag* bag) {
  for (const auto& m : rosbag::View(*bag)) {
    if (m.getTopic() == "/camera/depth/points") {
      auto s = m.instantiate<sensor_msgs::PointCloud2>();
      if (s == nullptr) {
        continue;
      }
      return PC16(*s);
    }
  }
  return {};
}
using PathSegment = std::pair<Eigen::Vector2f, Eigen::Vector2f>;
using PathSegments = std::vector<PathSegment>;

PathSegments GetPathSegments(const visualization_msgs::Marker& s) {
  static const auto to_eigen =
      [](const geometry_msgs::Point& p) -> Eigen::Vector2f {
    return {p.x, p.y};
  };
  PathSegments path;
  CHECK_EQ(s.points.size() % 2, 0);
  for (size_t i = 0; i < s.points.size() - 1; i += 2) {
    path.push_back({to_eigen(s.points[i]), to_eigen(s.points[i + 1])});
  }
  return path;
}

visualization_msgs::Marker GetPathMsg(rosbag::Bag* bag) {
  for (const auto& m : rosbag::View(*bag)) {
    if (m.getTopic() == "/base_link_robot_path") {
      auto s = m.instantiate<visualization_msgs::Marker>();
      if (s == nullptr) {
        continue;
      }
      if (s->points.empty()) {
        continue;
      }
      s->header.frame_id = "base_link";
      return *s;
    }
  }
  return {};
}

void FilterFloorPlane(PC16* pc) {
  for (auto& p : *pc) {
    if (p.z < 0.05) {
      p.Invalidate();
    }
  }
}
std::string vtos(const Eigen::Vector2f& v) {
  return std::to_string(v.x()) + ", " + std::to_string(v.y());
}

bool IsInFrontSegment(const Eigen::Vector2f& point, const PathSegment& line) {
  if (math_util::Sign(geometry::Cross(point, line.first)) !=
      -math_util::Sign(geometry::Cross(point, line.second))) {
    return false;
  }

  //  std::cout << vtos(line.first) << " -> " << vtos(line.second) << std::endl;

  // Line is in the format (from, to).
  const int line_direction = math_util::Sign(line.second.y() - line.first.y());
  const Eigen::Vector2f from_to_vector = line.second - line.first;
  const Eigen::Vector2f from_point_vector = point - line.first;
  const int directional_side =
      math_util::Sign(geometry::Cross(from_to_vector, from_point_vector));

  return (directional_side == line_direction);
}

void LabelNotInFrontOfPath(PC16* pc, const PathSegments& path_segments) {
  for (auto& p : *pc) {
    const Eigen::Vector2f v(p.x, p.y);
    bool in_front = false;
    for (const auto& ps : path_segments) {
      if (IsInFrontSegment(v, ps)) {
        in_front = true;
        break;
      }
    }

    p.pad = static_cast<std::uint32_t>(in_front);
  }

  for (size_t i = 100; i < 300; ++i) {
    for (auto& p : pc->RowIter(i)) {
      p.Invalidate();
    }
  }
}

visualization_msgs::Marker SetPath(const visualization_msgs::Marker& marker,
                                   const PathSegments& path_segments) {
  auto m = marker;
  m.points.clear();
  for (const auto& s : path_segments) {
    geometry_msgs::Point from;
    from.x = s.first.x();
    from.y = s.first.y();
    geometry_msgs::Point to;
    to.x = s.second.x();
    to.y = s.second.y();

    m.points.push_back(from);
    m.points.push_back(to);
  }
  return m;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bag_fill_in");
  auto res = GetArgs(argc, argv);
  rosbag::Bag bag(std::get<0>(res));
  PC16 pc = GetPC(&bag);
  auto path_msg = GetPathMsg(&bag);
  //  auto path_segments = GetPathSegments(path_msg);
  PathSegments path_segments = {
      {{0, 0}, {1.4, -0.3}}, {{1.4, -0.3}, {2, 0.4}}, {{2, 0.4}, {3, 0.7}}};
  path_msg = SetPath(path_msg, path_segments);

  static const Eigen::Affine3f transform =
      (Eigen::Affine3f::Identity() * Eigen::Translation3f(0, 0, 0.28) *
       Eigen::AngleAxisf(-kPi / 2, Eigen::Vector3f::UnitZ())) *
      Eigen::AngleAxisf(-kPi / 1.95, Eigen::Vector3f::UnitX());

  pc.TransformFrame(transform, "base_link");
  FilterFloorPlane(&pc);
  PC16 filtered_pc = pc;
  LabelNotInFrontOfPath(&filtered_pc, path_segments);

  ros::NodeHandle n;

  auto pc_pub = n.advertise<sensor_msgs::PointCloud2>("/pc", 10);
  auto filtered_pc_pub =
      n.advertise<sensor_msgs::PointCloud2>("/filtered_pc", 10);
  auto path_pub = n.advertise<visualization_msgs::Marker>("/path", 10);

  ros::Rate r(10);
  while (ros::ok()) {
    pc_pub.publish(*pc.GetRosPC());
    filtered_pc_pub.publish(*filtered_pc.GetRosPC());
    path_pub.publish(path_msg);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
