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

#include "cs/util/plane_fit.h"
#include "cs/util/point_cloud.h"
#include "cs/util/visualization.h"
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

std::string vtos(const Eigen::Vector3f& v) {
  return std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " +
         std::to_string(v.z());
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

void FilterNotInFrontOfPath(PC16* pc, const PathSegments& path_segments) {
  static constexpr std::uint32_t kPadKeep = 1;
  static constexpr std::uint32_t kPadDelete = 0;
  for (auto& p : *pc) {
    const Eigen::Vector2f v(p.x, p.y);
    bool in_front = false;
    for (const auto& ps : path_segments) {
      if (IsInFrontSegment(v, ps)) {
        in_front = true;
        break;
      }
    }

    p.pad = in_front ? kPadKeep : kPadDelete;
  }

  static constexpr int kNoIdxFound = -1;
  // Sweep horizontally
  for (int row_idx = 0; row_idx < static_cast<int>(pc->NumRows()); ++row_idx) {
    auto row_iter = pc->RowIter(row_idx);
    auto rev_row_iter = pc->RevRowIter(row_idx);
    int first_idx_found = kNoIdxFound;
    int last_idx_found = kNoIdxFound;
    int i = 0;
    for (auto& p : row_iter) {
      if (p.pad == kPadKeep) {
        last_idx_found = i;
        if (first_idx_found == kNoIdxFound) {
          first_idx_found = i;
        }
      }
      ++i;
    }

    if (last_idx_found == kNoIdxFound) {
      continue;
    }

    static constexpr float kMaxPlanarDistance = 0.5;
    static constexpr int kHorizontalMaxMissing = 10;

    static constexpr bool kGrowCluster = true;

    if (kGrowCluster) {
      // Sweep right
      const auto& last_point = *(row_iter.begin() + last_idx_found);
      NP_CHECK(last_point.pad == kPadKeep);
      const float last_point_dist =
          last_point.GetMappedVector2f().squaredNorm();
      int last_missing_count = 0;
      for (auto it = row_iter.begin() + last_idx_found; it != row_iter.end();
           ++it) {
        auto& query_point = *it;
        const float query_point_dist =
            query_point.GetMappedVector2f().squaredNorm();
        if (std::abs(query_point_dist - last_point_dist) < kMaxPlanarDistance) {
          query_point.pad = kPadKeep;
        } else {
          ++last_missing_count;
          if (last_missing_count > kHorizontalMaxMissing) {
            break;
          }
        }
      }

      // Sweep left
      const auto& first_point =
          *(rev_row_iter.begin() + (pc->NumColumns() - first_idx_found - 1));
      NP_CHECK(first_point.pad == kPadKeep);
      float current_point_dist = first_point.GetMappedVector2f().squaredNorm();
      int first_missing_count = 0;
      for (auto it =
               rev_row_iter.begin() + (pc->NumColumns() - first_idx_found);
           it != rev_row_iter.end();
           ++it) {
        auto& query_point = *it;
        const float query_point_dist =
            query_point.GetMappedVector2f().squaredNorm();
        if (std::abs(query_point_dist - current_point_dist) <
            kMaxPlanarDistance) {
          query_point.pad = kPadKeep;
          current_point_dist = query_point_dist;
        } else {
          ++first_missing_count;
          if (first_missing_count > kHorizontalMaxMissing) {
            break;
          }
        }
      }
    }
  }

  // Remove all not labeled as in front.

  for (auto& p : *pc) {
    if (p.pad == kPadDelete) {
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
      {{0, 0}, {1.4, -0.3}}, {{1.4, -0.3}, {2, 0.4}}, {{2, 0.4}, {3, 1.2}}};
  path_msg = SetPath(path_msg, path_segments);

  static const Eigen::Affine3f transform =
      (Eigen::Affine3f::Identity() * Eigen::Translation3f(0, 0, 0.28) *
       Eigen::AngleAxisf(-kPi / 2, Eigen::Vector3f::UnitZ())) *
      Eigen::AngleAxisf(-kPi / 1.95, Eigen::Vector3f::UnitX());

  pc.TransformFrame(transform, "base_link");
  FilterFloorPlane(&pc);
  PC16 filtered_pc = pc;
  FilterNotInFrontOfPath(&filtered_pc, path_segments);

  ros::NodeHandle n;

  auto pc_pub = n.advertise<sensor_msgs::PointCloud2>("/pc", 10);
  auto filtered_pc_pub =
      n.advertise<sensor_msgs::PointCloud2>("/filtered_pc", 10);
  auto path_pub = n.advertise<visualization_msgs::Marker>("/path", 10);
  auto plane_pub = n.advertise<visualization_msgs::Marker>("/plane", 10);

  ros::Rate r(10);
  while (ros::ok()) {
    pc_pub.publish(*pc.GetRosPC());
    filtered_pc_pub.publish(*filtered_pc.GetRosPC());
    path_pub.publish(path_msg);

    const auto plane = util::pca::FitPlane(filtered_pc);
    const auto plane_msg = visualization::DrawPlane(
        plane, "base_link", "ransac_plane", plane.ToQuaternion());
    plane_pub.publish(plane_msg);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
