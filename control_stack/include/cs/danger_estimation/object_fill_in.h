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
#include "cs/util/point_cloud.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

namespace cs {
namespace danger_estimation {

using PathSegment = std::pair<Eigen::Vector2f, Eigen::Vector2f>;
using PathSegments = std::vector<PathSegment>;

static constexpr std::uint32_t kPadObjectRing = 2;
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
PC<P> LabelObject(PC<P> pc,
                  const cs::danger_estimation::PathSegments& path_segments) {
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

template <typename P>
void FilterNotInFrontOfPath(
    PC<P>* pc, const cs::danger_estimation::PathSegments& path_segments) {
  auto labeled_pc = LabelObject(*pc, path_segments);
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
