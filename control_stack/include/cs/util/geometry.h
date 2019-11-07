#pragma once
// Copyright 2017 - 2018 joydeepb@cs.umass.edu, slane@cs.umass.edu,
// kvedder@seas.upenn.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// School of Engineering and Applied Sciences,
// University of Pennsylvania
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

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

#include "cs/util/constants.h"
#include "cs/util/math_util.h"
#include "cs/util/pose.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using math_util::AngleMod;
using math_util::Cos;
using math_util::IsAngleBetween;
using math_util::Sin;
using math_util::Sq;
using std::sqrt;

namespace geometry {

const float kEpsilon = 1e-6;

// Returns a unit vector in the direction of the given angle.
template <typename T>
Eigen::Matrix<T, 2, 1> Heading(T angle) {
  return Eigen::Matrix<T, 2, 1>(cos(angle), sin(angle));
}

// Returns a vector of the same magnitude, but rotated by 90 degrees counter-
// clockwise.
template <typename T>
Eigen::Matrix<T, 2, 1> Perp(const Eigen::Matrix<T, 2, 1>& v) {
  return Eigen::Matrix<T, 2, 1>(-v.y(), v.x());
}

// Returns the value of the 2D cross product between v1 and v2.
template <typename T>
T Cross(const Eigen::Matrix<T, 2, 1>& v1, const Eigen::Matrix<T, 2, 1>& v2) {
  return (v1.x() * v2.y() - v2.x() * v1.y());
}

template <typename T>
Eigen::Matrix<T, 2, 1> GetNormalizedOrZero(const Eigen::Matrix<T, 2, 1>& vec) {
  const auto norm = vec.template lpNorm<1>();
  if (norm == 0) {
    return vec;
  } else {
    return vec.normalized();
  }
}

template <typename T>
T GetNormOrZero(const Eigen::Matrix<T, 2, 1>& vec) {
  const auto norm = vec.template lpNorm<1>();
  if (norm == 0) {
    return 0;
  } else {
    return vec.template lpNorm<2>();
  }
}

// Returns true if v1 is parallel to v2
template <typename T>
bool IsParallel(const Eigen::Matrix<T, 2, 1>& v1,
                const Eigen::Matrix<T, 2, 1>& v2) {
  T l1 = v1.norm();
  T l2 = v2.norm();

  return fabs(fabs(v1.dot(v2)) - l1 * l2) < kEpsilon;
}

// Returns true if the line going from p1 to p2 is parallel to the line from p3
// to p4
template <typename T>
bool IsParallel(const Eigen::Matrix<T, 2, 1>& p1,
                const Eigen::Matrix<T, 2, 1>& p2,
                const Eigen::Matrix<T, 2, 1>& p3,
                const Eigen::Matrix<T, 2, 1>& p4) {
  const Eigen::Matrix<T, 2, 1> v1 = p2 - p1;
  const Eigen::Matrix<T, 2, 1> v2 = p4 - p3;
  const T l1 = v1.norm();
  const T l2 = v2.norm();

  return fabs(fabs(v1.dot(v2)) - l1 * l2) < kEpsilon;
}

// Returns true if v1 is perpendicular to v2
template <typename T>
bool IsPerpendicular(const Eigen::Matrix<T, 2, 1>& v1,
                     const Eigen::Matrix<T, 2, 1>& v2) {
  return fabs(v1.dot(v2)) < kEpsilon;
}

// Returns the two tangent points t0, t1 from the point p, to a circle with
// center c, with radius r.
template <typename T>
void GetTangentPoints(const Eigen::Matrix<T, 2, 1>& p,
                      const Eigen::Matrix<T, 2, 1>& c, const T& r,
                      Eigen::Matrix<T, 2, 1>* right_tangent,
                      Eigen::Matrix<T, 2, 1>* left_tangent) {
  DCHECK_NE(right_tangent, (static_cast<Eigen::Matrix<T, 2, 1>*>(nullptr)));
  DCHECK_NE(left_tangent, (static_cast<Eigen::Matrix<T, 2, 1>*>(nullptr)));

  // Vector from c to p, which will then be normalized.
  Eigen::Matrix<T, 2, 1> c_p_dir = c - p;
  // Squared distance of c to p.
  const T c_p_sqdist = c_p_dir.squaredNorm();
  // Distance of c to p.
  const T c_p_dist = sqrt(c_p_sqdist);
  c_p_dir = c_p_dir / c_p_dist;
  // Distance of tangent point to p.
  const T t_p_dist = sqrt(c_p_sqdist - math_util::Sq(r));
  // Unit vector perpendicular to the line joining c and p.
  const Eigen::Matrix<T, 2, 1> c_p_perp = Perp(c_p_dir);
  // Cosine of the angle between the line from c to the tangent point t0, and
  // the line joining c to p.
  const T cos_t = r / c_p_dist;
  // Sine  of the angle between the line from c to the tangent point t0, and
  // the line joining c to p.
  const T sin_t = t_p_dist / c_p_dist;
  // Projected location of the tangent points onto the line joining c and p.
  const Eigen::Matrix<T, 2, 1> t_projected = c - cos_t * r * c_p_dir;
  *right_tangent = t_projected - sin_t * r * c_p_perp;
  *left_tangent = t_projected + sin_t * r * c_p_perp;
}

// Returns true if point c is between points a and b on a line.
template <typename T>
bool IsBetween(const Eigen::Matrix<T, 2, 1>& a, const Eigen::Matrix<T, 2, 1>& b,
               const Eigen::Matrix<T, 2, 1>& c, const T epsilon) {
  const T dist = (a - c).norm() + (c - b).norm() - (a - b).norm();
  return dist <= epsilon;
}

// Checks to see if a line line collision exists between the lines given
// by (a1-a0) and (b1-b0).
template <typename T>
bool CheckLineLineCollision(const Eigen::Matrix<T, 2, 1>& a0,
                            const Eigen::Matrix<T, 2, 1>& a1,
                            const Eigen::Matrix<T, 2, 1>& b0,
                            const Eigen::Matrix<T, 2, 1>& b1) {
  const Eigen::Matrix<T, 2, 1> a1a0 = a1 - a0;
  const Eigen::Matrix<T, 2, 1> b0a0 = b0 - a0;
  const Eigen::Matrix<T, 2, 1> b1a0 = b1 - a0;
  const Eigen::Matrix<T, 2, 1> b1b0 = b1 - b0;
  const Eigen::Matrix<T, 2, 1> a0b0 = a0 - b0;
  const Eigen::Matrix<T, 2, 1> a1b0 = a1 - b0;
  return (math_util::Sign(Cross(a1a0, b0a0)) !=
          math_util::Sign(Cross(a1a0, b1a0))) &&
         (math_util::Sign(Cross(b1b0, a0b0)) !=
          math_util::Sign(Cross(b1b0, a1b0)));
}

// Returns the point of intersection between the lines given by (a1-a0) and
// (b1-b0).
template <typename T>
Eigen::Matrix<T, 2, 1> LineLineIntersection(const Eigen::Matrix<T, 2, 1>& a0,
                                            const Eigen::Matrix<T, 2, 1>& a1,
                                            const Eigen::Matrix<T, 2, 1>& b0,
                                            const Eigen::Matrix<T, 2, 1>& b1) {
  // Vector form of solution taken from:
  // http://mathworld.wolfram.com/Line-LineIntersection.html
  const auto& x1 = a0;
  const auto& x2 = a1;
  const auto& x3 = b0;
  const auto& x4 = b1;
  const Eigen::Matrix<T, 2, 1> a = (x2 - x1);
  const Eigen::Matrix<T, 2, 1> b = (x4 - x3);
  const Eigen::Matrix<T, 2, 1> c = (x3 - x1);
  // Simplifies to remove one call of Cross(a, b) from the numerator and
  // denominator.
  const T numerator = Cross(c, b);
  const T denominator = Cross(a, b);
  return x1 + a * (numerator / denominator);
}

// Checks for intersection between the lines given by (a1-a0) and
// (b1-b0).
//
// If one is found, then it will be returned as the second of the pair.
// If one is not found, then an arbitrary point will be returned.
template <typename T>
std::pair<bool, Eigen::Matrix<T, 2, 1>> CheckLineLineIntersection(
    const Eigen::Matrix<T, 2, 1>& a0, const Eigen::Matrix<T, 2, 1>& a1,
    const Eigen::Matrix<T, 2, 1>& b0, const Eigen::Matrix<T, 2, 1>& b1) {
  if (!CheckLineLineCollision(a0, a1, b0, b1)) {
    return {false,
            {std::numeric_limits<T>::max(), std::numeric_limits<T>::max()}};
  }
  return {true, LineLineIntersection(a0, a1, b0, b1)};
}

// Returns the angle corresponding to the direction of this vector.
template <typename T>
T Angle(const Eigen::Matrix<T, 2, 1>& v) {
  return (atan2(v.y(), v.x()));
}

// Project a given point onto a line segment.
// The line is defined by its two vertices vertex_a and vertex_b
template <typename T>
Eigen::Matrix<T, 2, 1> ProjectPointOntoLine(
    const Eigen::Matrix<T, 2, 1>& point, const Eigen::Matrix<T, 2, 1>& vertex_a,
    const Eigen::Matrix<T, 2, 1>& vertex_b) {
  Eigen::Matrix<T, 2, 1> line_segment = vertex_b - vertex_a;
  Eigen::Matrix<T, 2, 1> point_vector = point - vertex_a;

  // Project the point onto the line segment, capping it at the two end points
  const float scalar_projection =
      point_vector.dot(line_segment) / line_segment.squaredNorm();

  return vertex_a + scalar_projection * line_segment;
}

// Project a given point onto a line segment.
// The line segment is defined by its two vertices vertex_a and vertex_b
template <typename T>
Eigen::Matrix<T, 2, 1> ProjectPointOntoLineSegment(
    const Eigen::Matrix<T, 2, 1>& point, const Eigen::Matrix<T, 2, 1>& vertex_a,
    const Eigen::Matrix<T, 2, 1>& vertex_b) {
  Eigen::Matrix<T, 2, 1> line_segment = vertex_b - vertex_a;
  Eigen::Matrix<T, 2, 1> point_vector = point - vertex_a;

  // Project the point onto the line segment, capping it at the two end points
  float scalar_projection =
      point_vector.dot(line_segment) / line_segment.squaredNorm();

  scalar_projection = fmax(0, fmin(1, scalar_projection));

  return vertex_a + scalar_projection * line_segment;
}

template <typename T>
T MinDistanceLineLine(const Eigen::Matrix<T, 2, 1>& a0,
                      const Eigen::Matrix<T, 2, 1>& a1,
                      const Eigen::Matrix<T, 2, 1>& b0,
                      const Eigen::Matrix<T, 2, 1>& b1) {
  if (CheckLineLineCollision(a0, a1, b0, b1)) {
    return 0;
  }

  const auto a0_proj = ProjectPointOntoLineSegment(a0, b0, b1);
  const auto a1_proj = ProjectPointOntoLineSegment(a1, b0, b1);
  const auto b0_proj = ProjectPointOntoLineSegment(b0, a0, a1);
  const auto b1_proj = ProjectPointOntoLineSegment(b1, a0, a1);

  const auto a0_diff_sq = (a0 - a0_proj).squaredNorm();
  const auto a1_diff_sq = (a1 - a1_proj).squaredNorm();
  const auto b0_diff_sq = (b0 - b0_proj).squaredNorm();
  const auto b1_diff_sq = (b1 - b1_proj).squaredNorm();

  const auto min_diff_sq =
      std::min({a0_diff_sq, a1_diff_sq, b0_diff_sq, b1_diff_sq});
  return std::sqrt(min_diff_sq);
}

// Check if line segment collides min and max angle on a circle, moving in a
// counterclockwise direction from min_angle to max_angle.
template <typename T>
T MinDistanceLineArc(const Eigen::Matrix<T, 2, 1>& l0,
                     const Eigen::Matrix<T, 2, 1>& l1,
                     const Eigen::Matrix<T, 2, 1>& a_center, const T& a_radius,
                     T a_angle_start, T a_angle_end, const int rotation_sign) {
  static constexpr bool kDebug = false;
  NP_FINITE_2F(l0);
  NP_FINITE_2F(l1);
  NP_FINITE_2F(a_center);
  NP_FINITE(a_angle_start);
  NP_FINITE(a_angle_end);
  NP_CHECK_VAL(a_radius >= 0, a_radius);
  NP_CHECK_VAL(rotation_sign == 0 || rotation_sign == 1 || rotation_sign == -1,
               rotation_sign);
  a_angle_start = AngleMod(a_angle_start);
  a_angle_end = AngleMod(a_angle_end);
  const auto proj_center_segment =
      ProjectPointOntoLineSegment(a_center, l0, l1);
  const auto proj_dir_segment = proj_center_segment - a_center;

  const auto l0_dir = l0 - a_center;
  const auto l1_dir = l1 - a_center;

  if (proj_dir_segment.squaredNorm() >= Sq(a_radius) ||
      (l0_dir.squaredNorm() < Sq(a_radius) &&
       l1_dir.squaredNorm() < Sq(a_radius))) {
    if (kDebug) {
      std::cout
          << "Out of the circle, tangent to circle, or completely in circle."
          << std::endl;
    }
    // Out of the circle, tangent to circle, or completely in circle.
    const T angle = AngleMod(Angle<T>(proj_dir_segment));
    NP_FINITE(angle);
    if (IsAngleBetween(angle, a_angle_start, a_angle_end, rotation_sign)) {
      // Vector of the center projected onto line segment is in arc.
      const float distance_to_proj = (proj_dir_segment.norm() - a_radius);
      if (distance_to_proj >= 0) {
        return distance_to_proj;
      }
    }
    // Vector of the center projected onto line segment is not in arc.
    const auto a_angle_min_v = Heading(a_angle_start) * a_radius;
    const auto a_angle_max_v = Heading(a_angle_end) * a_radius;
    NP_FINITE_2F(a_angle_min_v);
    NP_FINITE_2F(a_angle_max_v);
    return std::sqrt(
        std::min((proj_dir_segment - a_angle_min_v).squaredNorm(),
                 (proj_dir_segment - a_angle_max_v).squaredNorm()));
  }

  const auto proj_center_line = ProjectPointOntoLine(a_center, l0, l1);
  NP_FINITE_2F(proj_center_line);
  const auto proj_dir_line = proj_center_line - a_center;
  NP_FINITE_2F(proj_dir_line);
  const T along_line_dist =
      std::sqrt(Sq(a_radius) - proj_dir_line.squaredNorm());
  NP_FINITE(along_line_dist);

  static const auto direction_line_intersects_arc =
      [](const Eigen::Matrix<T, 2, 1>& proj_center_line,
         const T& along_line_dist, const T& a_angle_start, const T& a_angle_end,
         const Eigen::Matrix<T, 2, 1>& a_center, const int& rotation_sign,
         const Eigen::Matrix<T, 2, 1>& direction_line) -> bool {
    NP_CHECK_VAL(
        rotation_sign == 0 || rotation_sign == 1 || rotation_sign == -1,
        rotation_sign);
    const auto circle_intersect_point =
        direction_line * along_line_dist + proj_center_line;
    if (kDebug) {
      std::cout << "Circle intersect point: " << circle_intersect_point.x()
                << ", " << circle_intersect_point.y() << std::endl;
    }
    NP_FINITE_2F(circle_intersect_point);
    const T angle = AngleMod(Angle<T>(circle_intersect_point - a_center));
    NP_FINITE(angle);
    if (kDebug) {
      std::cout << "Query angle: " << angle << " Min angle: " << a_angle_start
                << " Max angle: " << a_angle_end << " Sign: " << rotation_sign
                << std::endl;
      std::cout << "Is between: "
                << (IsAngleBetween(angle, a_angle_start, a_angle_end,
                                   rotation_sign)
                        ? "true"
                        : "false")
                << std::endl;
    }
    return IsAngleBetween(angle, a_angle_start, a_angle_end, rotation_sign);
  };

  static const auto no_arc_intersect_min_distance =
      [](const Eigen::Matrix<T, 2, 1>& a_center, const T& a_angle_start,
         const T& a_angle_end, const T& a_radius,
         const Eigen::Matrix<T, 2, 1>& l0,
         const Eigen::Matrix<T, 2, 1>& l1) -> float {
    const auto a_angle_min_v = Heading(a_angle_start) * a_radius + a_center;
    NP_FINITE_2F(a_angle_min_v);
    //    std::cout << "a_angle_min_v: " << a_angle_min_v.x() << ", "
    //              << a_angle_min_v.y() << std::endl;
    const auto a_angle_max_v = Heading(a_angle_end) * a_radius + a_center;
    NP_FINITE_2F(a_angle_max_v);
    //    std::cout << "a_angle_max_v: " << a_angle_max_v.x() << ", "
    //              << a_angle_max_v.y() << std::endl;
    const auto proj_a_angle_min_v =
        ProjectPointOntoLineSegment<T>(a_angle_min_v, l0, l1);
    NP_FINITE_2F(proj_a_angle_min_v);
    //    std::cout << "proj_a_angle_min_v " << proj_a_angle_min_v.x() << ", "
    //              << proj_a_angle_min_v.y() << std::endl;
    const auto proj_a_angle_max_v =
        ProjectPointOntoLineSegment<T>(a_angle_max_v, l0, l1);
    NP_FINITE_2F(proj_a_angle_max_v);
    //    std::cout << "proj_a_angle_max_v " << proj_a_angle_max_v.x() << ", "
    //              << proj_a_angle_max_v.y() << std::endl;
    const Eigen::Matrix<T, 2, 1> del_min_v = proj_a_angle_min_v - a_angle_min_v;
    NP_FINITE_2F(del_min_v);
    const T del_min_v_sq_norm = del_min_v.squaredNorm();
    NP_FINITE(del_min_v_sq_norm);
    const Eigen::Matrix<T, 2, 1> del_max_v = proj_a_angle_max_v - a_angle_max_v;
    NP_FINITE_2F(del_max_v);
    const T del_max_v_sq_norm = del_max_v.squaredNorm();
    NP_FINITE(del_max_v_sq_norm);
    if (kDebug) {
      std::cout << "no_arc_intersect_min_distances: del_min_v "
                << std::sqrt(del_min_v_sq_norm) << " " << del_min_v.norm()
                << " del_max_v " << std::sqrt(del_max_v_sq_norm) << " "
                << del_max_v.norm() << std::endl;
    }
    const T min_dist =
        std::sqrt(std::min(del_min_v_sq_norm, del_max_v_sq_norm));
    NP_FINITE(min_dist);
    return min_dist;
  };

  // l0 inside the circle, l1 outside the circle.
  if (l0_dir.squaredNorm() < Sq(a_radius) &&
      l1_dir.squaredNorm() >= Sq(a_radius)) {
    if (kDebug) {
      std::cout << "l0 inside the circle, l1 outside the circle." << std::endl;
      std::cout << "l0: " << l0.x() << ", " << l0.y() << std::endl;
      std::cout << "l1: " << l1.x() << ", " << l1.y() << std::endl;
    }
    const auto direction_line =
        GetNormalizedOrZero(Eigen::Matrix<T, 2, 1>(l1 - l0));
    NP_FINITE_2F(direction_line);
    if (direction_line_intersects_arc(proj_center_line, along_line_dist,
                                      a_angle_start, a_angle_end, a_center,
                                      rotation_sign, direction_line)) {
      // Arc intersects with line.
      return 0;
    }

    return no_arc_intersect_min_distance(a_center, a_angle_start, a_angle_end,
                                         a_radius, l0, l1);
  }

  // l1 inside the circle, l0 outside the circle.
  if (l0_dir.squaredNorm() >= Sq(a_radius) &&
      l1_dir.squaredNorm() < Sq(a_radius)) {
    if (kDebug) {
      std::cout << "l1 inside the circle, l0 outside the circle." << std::endl;
      std::cout << "l0: " << l0.x() << ", " << l0.y() << std::endl;
      std::cout << "l1: " << l1.x() << ", " << l1.y() << std::endl;
    }
    const auto direction_line =
        GetNormalizedOrZero(Eigen::Matrix<T, 2, 1>(l0 - l1));
    NP_FINITE_2F(direction_line);
    if (direction_line_intersects_arc(proj_center_line, along_line_dist,
                                      a_angle_start, a_angle_end, a_center,
                                      rotation_sign, direction_line)) {
      // Arc intersects with line.
      return 0;
    }
    return no_arc_intersect_min_distance(a_center, a_angle_start, a_angle_end,
                                         a_radius, l0, l1);
  }

  // Both l0 and l1 are not in the circle, but the line goes through the circle.
  if (kDebug) {
    std::cout << "Both l0 and l1 are not in the circle, but the line goes "
                 "through the circle."
              << std::endl;
  }
  const auto direction_line =
      GetNormalizedOrZero(Eigen::Matrix<T, 2, 1>(l1 - l0));
  NP_FINITE_2F(direction_line);

  const bool positive_direction_intersect = direction_line_intersects_arc(
      proj_center_line, along_line_dist, a_angle_start, a_angle_end, a_center,
      rotation_sign, direction_line);
  const bool negative_direction_intersect = direction_line_intersects_arc(
      proj_center_line, along_line_dist, a_angle_start, a_angle_end, a_center,
      rotation_sign, -direction_line);

  if (positive_direction_intersect || negative_direction_intersect) {
    // Arc intersects with line.
    if (kDebug) {
      std::cout << "Arcs intersect" << std::endl;
    }
    return 0;
  }

  const T min_distance = no_arc_intersect_min_distance(
      a_center, a_angle_start, a_angle_end, a_radius, l0, l1);
  if (kDebug) {
    std::cout << "Min distance: " << min_distance << std::endl;
  }
  return min_distance;
}

util::Pose FollowTrajectory(const util::Pose& pose_global_frame,
                            const float& distance_along_arc,
                            const float& rotation) {
  const Eigen::Rotation2Df robot_to_global_frame(pose_global_frame.rot);
  const Eigen::Vector2f robot_forward_global_frame =
      robot_to_global_frame * Eigen::Vector2f(1, 0);

  if (rotation == 0) {
    util::Pose updated_pose = pose_global_frame;
    updated_pose.tra += robot_forward_global_frame * distance_along_arc;
    return updated_pose;
  }

  const float circle_radius = distance_along_arc / rotation;

  const float move_x_dist = Sin(rotation) * circle_radius;
  const float move_y_dist =
      (Cos(fabs(rotation)) * circle_radius - circle_radius);

  const Eigen::Vector2f movement_arc_robot_frame(move_x_dist, move_y_dist);
  const Eigen::Vector2f movement_arc_global_frame =
      robot_to_global_frame * movement_arc_robot_frame;

  return {movement_arc_global_frame + pose_global_frame.tra,
          AngleMod(rotation + pose_global_frame.rot)};
}

}  // namespace geometry
