#pragma once
// Copyright 2020 kvedder@seas.upenn.edu
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

#include <algorithm>
#include <array>
#include <limits>
#include <utility>
#include <vector>

#include "cs/danger_estimation/object_library.h"
#include "cs/util/constants.h"
#include "cs/util/plane_fit.h"
#include "cs/util/point_cloud.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

namespace cs {
namespace danger_estimation {

using PathSegment = std::pair<Eigen::Vector2f, Eigen::Vector2f>;
using PathSegments = std::vector<PathSegment>;

static constexpr std::uint32_t kPadObject = 1;
static constexpr std::uint32_t kPadNoObject = 0;

template <typename P>
using PC = util::pc::PointCloud<P>;

bool IsInFrontSegment(const Eigen::Vector2f& point, const PathSegment& line) {
  if (math_util::Sign(geometry::Cross(point, line.first)) !=
      -math_util::Sign(geometry::Cross(point, line.second))) {
    return false;
  }

  // Line is in the format (from, to).
  const int line_direction = math_util::Sign(line.second.y() - line.first.y());
  const Eigen::Vector2f from_to_vector = line.second - line.first;
  const Eigen::Vector2f from_point_vector = point - line.first;
  const int directional_side =
      math_util::Sign(geometry::Cross(from_to_vector, from_point_vector));

  return (directional_side == line_direction);
}

template <typename P>
PC<P> LabelPCWithObject(
    PC<P> pc, const cs::danger_estimation::PathSegments& path_segments) {
  for (auto& p : pc) {
    const Eigen::Vector2f v(p.x, p.y);
    bool in_front = false;
    for (const auto& ps : path_segments) {
      if (IsInFrontSegment(v, ps)) {
        in_front = true;
        break;
      }
    }
    p.pad = in_front ? kPadObject : kPadNoObject;
  }

  static constexpr int kNoIdxFound = -1;
  // Sweep horizontally
  for (int row_idx = 0; row_idx < static_cast<int>(pc.NumRows()); ++row_idx) {
    auto row_iter = pc.RowIter(row_idx);
    auto rev_row_iter = pc.RevRowIter(row_idx);
    int first_idx_found = kNoIdxFound;
    int last_idx_found = kNoIdxFound;
    int i = 0;
    for (auto& p : row_iter) {
      if (p.pad == kPadObject) {
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
      NP_CHECK(last_point.pad == kPadObject);
      const float last_point_dist =
          last_point.GetMappedVector2f().squaredNorm();
      int last_missing_count = 0;
      for (auto it = row_iter.begin() + last_idx_found; it != row_iter.end();
           ++it) {
        auto& query_point = *it;
        const float query_point_dist =
            query_point.GetMappedVector2f().squaredNorm();
        if (query_point.IsValid() &&
            std::abs(query_point_dist - last_point_dist) < kMaxPlanarDistance) {
          query_point.pad = kPadObject;
        } else {
          ++last_missing_count;
          if (last_missing_count > kHorizontalMaxMissing) {
            break;
          }
        }
      }

      // Sweep left
      const auto& first_point =
          *(rev_row_iter.begin() + (pc.NumColumns() - first_idx_found - 1));
      NP_CHECK(first_point.pad == kPadObject);
      float current_point_dist = first_point.GetMappedVector2f().squaredNorm();
      int first_missing_count = 0;
      for (auto it = rev_row_iter.begin() + (pc.NumColumns() - first_idx_found);
           it != rev_row_iter.end();
           ++it) {
        auto& query_point = *it;
        const float query_point_dist =
            query_point.GetMappedVector2f().squaredNorm();
        if (query_point.IsValid() &&
            std::abs(query_point_dist - current_point_dist) <
                kMaxPlanarDistance) {
          query_point.pad = kPadObject;
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
  return pc;
}

struct ClosestPositions {
  bool initialized;
  Eigen::Vector2f bisecting_line;
  float travel_bisecting;
  float travel_perp;
  Eigen::Vector2f left_side;
  Eigen::Vector2f right_side;

  ClosestPositions()
      : initialized(false),
        bisecting_line(Eigen::Vector2f::Zero()),
        travel_bisecting(0),
        travel_perp(0),
        left_side(Eigen::Vector2f::Zero()),
        right_side(Eigen::Vector2f::Zero()) {}

  ClosestPositions(const Eigen::Vector2f& bisecting_line,
                   const float travel_bisecting,
                   const float travel_perp,
                   const Eigen::Vector2f& left_side,
                   const Eigen::Vector2f& right_side)
      : initialized(true),
        bisecting_line(bisecting_line),
        travel_bisecting(travel_bisecting),
        travel_perp(travel_perp),
        left_side(left_side),
        right_side(right_side) {}
};

ClosestPositions ObjectClosestPositions(
    const util::Plane& plane,
    const ObjectDescription& object_description,
    const float sensor_height) {
  if (!plane.initialized) {
    return {};
  }
  static constexpr auto v3tov2 =
      [](const Eigen::Vector3f& v3) -> Eigen::Vector2f {
    return {v3.x(), v3.y()};
  };
  const auto corners = plane.ToCorners();
  const auto upper_left = v3tov2(corners.upper_left);
  const auto upper_right = v3tov2(corners.upper_right);
  const auto lower_left = v3tov2(corners.lower_left);
  const auto lower_right = v3tov2(corners.lower_right);

  static constexpr auto pick_left =
      [](const Eigen::Vector2f& v1,
         const Eigen::Vector2f& v2) -> Eigen::Vector2f {
    if (v1.y() < v2.y()) {
      return v1;
    }
    return v2;
  };
  static constexpr auto pick_right =
      [](const Eigen::Vector2f& v1,
         const Eigen::Vector2f& v2) -> Eigen::Vector2f {
    if (v1.y() < v2.y()) {
      return v2;
    }
    return v1;
  };

  auto get_top_slope = [&sensor_height](const Eigen::Vector3f& v1,
                                        const Eigen::Vector3f& v2) -> float {
    const Eigen::Vector3f v1_moved = v1 - Eigen::Vector3f(0, 0, sensor_height);
    const Eigen::Vector3f v2_moved = v2 - Eigen::Vector3f(0, 0, sensor_height);
    return std::min(v1_moved.normalized().z(), v2_moved.normalized().z());
  };

  const Eigen::Vector2f left_side =
      pick_left(upper_left.normalized(), lower_left.normalized());
  const Eigen::Vector2f right_side =
      pick_right(upper_right.normalized(), lower_right.normalized());
  const Eigen::Vector2f bisecting_line =
      (left_side / 2 + right_side / 2).normalized();

  const float side_bisecting_angle = std::acos(bisecting_line.dot(left_side));

  const float distance_along_bisecting_sides_constraint =
      1.0 / std::sin(side_bisecting_angle) * object_description.radius;
  const float top_slope =
      get_top_slope(corners.upper_left, corners.upper_right);
  NP_CHECK_MSG(top_slope >= 0, "TODO add support for negative slopes");
  const float distance_along_bisecting_top_constraint =
      (object_description.height - sensor_height) / top_slope;

  if (distance_along_bisecting_sides_constraint >
      distance_along_bisecting_top_constraint) {
    return {bisecting_line,
            distance_along_bisecting_sides_constraint,
            0,
            left_side,
            right_side};
  }

  const float travel_perp_distance =
      std::sin(side_bisecting_angle) * distance_along_bisecting_top_constraint -
      object_description.radius;
  return {bisecting_line,
          distance_along_bisecting_top_constraint,
          travel_perp_distance,
          left_side,
          right_side};
}

bool PathSegmentCollides(const ClosestPositions& closest_positions,
                         const ObjectDescription& object_description,
                         const PathSegment& path_segment) {
  if (!closest_positions.initialized) {
    return false;
  }
  const Eigen::Vector2f& path_start = path_segment.first;
  const Eigen::Vector2f& path_end = path_segment.second;

  const Eigen::Vector2f bisecting_line_perp_left =
      Eigen::Rotation2Df(kPi / 2) * closest_positions.bisecting_line;
  const Eigen::Vector2f center =
      closest_positions.bisecting_line * closest_positions.travel_bisecting;
  const Eigen::Vector2f left_start =
      center + bisecting_line_perp_left * closest_positions.travel_perp;
  const Eigen::Vector2f right_start =
      center - bisecting_line_perp_left * closest_positions.travel_perp;

  if (closest_positions.travel_perp > 0) {
    const float min_distance = geometry::MinDistanceLineLine(
        left_start, right_start, path_start, path_end);
    if (min_distance < object_description.radius) {
      std::cout << "Perp collide" << std::endl;
      return true;
    }
  }

  static constexpr float kSideScale = 100;
  const Eigen::Vector2f left_delta = closest_positions.left_side * kSideScale;
  const Eigen::Vector2f left_end = left_start + left_delta;

  const Eigen::Vector2f right_delta = closest_positions.right_side * kSideScale;
  const Eigen::Vector2f right_end = right_start + right_delta;

  const float min_distance_left =
      geometry::MinDistanceLineLine(left_start, left_end, path_start, path_end);
  if (min_distance_left <= object_description.radius) {
    std::cout << "Left wall collide" << std::endl;
    return true;
  }
  const float min_distance_right = geometry::MinDistanceLineLine(
      right_start, right_end, path_start, path_end);
  if (min_distance_right <= object_description.radius) {
    std::cout << "Right wall collide" << std::endl;
    return true;
  }

  const auto point_inside_trapazoid =
      [&closest_positions,
       &center,
       &left_start,
       &right_start,
       &left_delta,
       &right_delta](const Eigen::Vector2f& point) -> bool {
    const Eigen::Vector2f center_point = point - center;

    if (closest_positions.bisecting_line.dot(center_point) < 0) {
      return false;
    }

    const Eigen::Vector2f left_point = point - left_start;
    const Eigen::Vector2f right_point = point - right_start;

    return math_util::Sign(geometry::Cross(left_point, left_delta)) ==
           math_util::Sign(geometry::Cross(right_delta, right_point));
  };

  if (point_inside_trapazoid(path_start)) {
    std::cout << "Inside start" << std::endl;
    return true;
  }
  if (point_inside_trapazoid(path_end)) {
    std::cout << "Inside end" << std::endl;
    return true;
  }
  return false;
}

template <typename P>
void FilterNotInFrontOfPath(PC<P>* pc, const PathSegments& path_segments) {
  auto labeled_pc = LabelPCWithObject(*pc, path_segments);
  //  labeled_pc = LabelObjectRing(labeled_pc);
  *pc = labeled_pc;
  for (auto& p : *pc) {
    if (p.pad == kPadNoObject) {
      p.Invalidate();
    }
  }
}

}  // namespace danger_estimation
}  // namespace cs
