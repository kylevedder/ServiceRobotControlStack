#pragma once
//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    map.h
\brief   Vector map representation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>
#include "cs/util/pose.h"
#include "shared/math/line2d.h"

namespace util {

using Wall = geometry::Line2f;

namespace vector_map {

// Checks if any part of trim_line is occluded by test_line when seen from
// loc, and if so, trim_line is trimmed accordingly, adding sub-lines to
// scene_lines if necessary.
void TrimOcclusion(const Eigen::Vector2f& loc,
                   const geometry::Line2f& line1,
                   geometry::Line2f* line2_ptr,
                   std::vector<geometry::Line2f>* scene_lines_ptr);

struct VectorMap {
  VectorMap() {}
  explicit VectorMap(const std::vector<geometry::Line2f>& lines)
      : lines(lines) {}
  explicit VectorMap(const std::string& file) { Load(file); }

  void GetSceneLines(const Eigen::Vector2f& loc,
                     float max_range,
                     std::vector<geometry::Line2f>* lines_list) const;

  void SceneRender(const Eigen::Vector2f& loc,
                   float max_range,
                   float angle_min,
                   float angle_max,
                   std::vector<geometry::Line2f>* render) const;

  void RayCast(const Eigen::Vector2f& loc,
               float max_range,
               std::vector<geometry::Line2f>* render) const;

  // Get predicted laser scan from current location.
  void GetPredictedScan(const Eigen::Vector2f& loc,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max,
                        int num_rays,
                        std::vector<float>* scan) const;
  void Cleanup();

  void Load(const std::string& file);

  bool Intersects(const Eigen::Vector2f& v0, const Eigen::Vector2f& v1) const;
  std::vector<geometry::Line2f> lines;

  // for all kinds of obstacles
  std::vector<geometry::Line2f> object_lines;
  std::string file_name;
};

}  // namespace vector_map

}  // namespace util
