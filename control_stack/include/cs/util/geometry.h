#pragma once

// Copyright 2017 - 2018 joydeepb@cs.umass.edu, slane@cs.umass.edu,
// kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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

#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

#include "constants.h"
#include "cs/util/math_util.h"
#include "cs/util/pose.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using math_util::AngleMod;
using math_util::Cos;
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

// Project a given point onto the line passing through vertex_a and vertex_b.
template <typename T>
void ProjectPointOntoLine(const Eigen::Matrix<T, 2, 1>& point,
                          const Eigen::Matrix<T, 2, 1>& vertex_a,
                          const Eigen::Matrix<T, 2, 1>& vertex_b,
                          Eigen::Matrix<T, 2, 1>* projected_point) {
  const Eigen::Matrix<T, 2, 1> line_dir = (vertex_b - vertex_a).normalized();
  // const Eigen::Matrix<T, 2, 1> line_perp = Perp<T>(line_dir);
  *projected_point = vertex_a + line_dir * line_dir.dot(point - vertex_a);
}

// Project a given point onto a line segment.
// The line segment is defined by its two vertices vertex_a and vertex_b
// The pointers projected_point and distance define the return values
// Projected point is the location ot the projection.
// Squared Distance is the squared distance from the original point to the
// starting point.
template <typename T>
void ProjectPointOntoLineSegment(const Eigen::Matrix<T, 2, 1>& point,
                                 const Eigen::Matrix<T, 2, 1>& vertex_a,
                                 const Eigen::Matrix<T, 2, 1>& vertex_b,
                                 Eigen::Matrix<T, 2, 1>* projected_point,
                                 float* squared_distance) {
  Eigen::Matrix<T, 2, 1> line_segment = vertex_b - vertex_a;
  Eigen::Matrix<T, 2, 1> point_vector = point - vertex_a;

  // Project the point onto the line segment, capping it at the two end points
  float scalar_projection =
      point_vector.dot(line_segment) / line_segment.squaredNorm();

  scalar_projection = fmax(0, fmin(1, scalar_projection));

  *projected_point = vertex_a + scalar_projection * line_segment;
  *squared_distance = (point - (*projected_point)).squaredNorm();
}

// Determines whether a ray intersects with a given line segment.
// Returns true if they intersect and false otherwise.
// ray_soruce is the origin point of the ray_direction
// ray_direction is a vector that indicates the direction that the ray is going
// in
// line_start is the start point of the line segment
// line_end is the end point of the line segment
// distance is defined to be the distance from the ray source to the intersect
// If there is no intersection, it will be set to -1
// intersect_point is the point at which the ray intersects the line segment
// It will be set to -1, -1 if there is no collision
template <typename T>
bool RayIntersect(const Eigen::Matrix<T, 2, 1>& ray_source,
                  const Eigen::Matrix<T, 2, 1>& ray_direction,
                  const Eigen::Matrix<T, 2, 1>& segment_start,
                  const Eigen::Matrix<T, 2, 1>& segment_end,
                  T* squared_distance,
                  Eigen::Matrix<T, 2, 1>* intersect_point) {
  bool intersects = false;
  *squared_distance = -1;
  intersect_point->x() = -1;
  intersect_point->y() = -1;

  Eigen::Matrix<T, 2, 1> line_to_source = ray_source - segment_start;
  Eigen::Matrix<T, 2, 1> line_direction = segment_end - segment_start;
  Eigen::Matrix<T, 2, 1> perpendicular = Perp(ray_direction);

  T denomenator = line_direction.dot(perpendicular);

  T ray_intersect_param = Cross(line_direction, line_to_source) / denomenator;

  T line_intersect_param = line_to_source.dot(perpendicular) / denomenator;

  if (ray_intersect_param >= 0 && line_intersect_param >= 0 &&
      line_intersect_param <= 1) {
    intersects = true;
    *intersect_point = (segment_start + line_intersect_param * line_direction);
    *squared_distance = ((*intersect_point) - ray_source).squaredNorm();
  }
  return intersects;
}

template <typename T>
bool RayIntersect(const Eigen::Matrix<T, 2, 1>& ray_source,
                  const Eigen::Matrix<T, 2, 1>& ray_direction,
                  const Eigen::Matrix<T, 2, 1>& line_start,
                  const Eigen::Matrix<T, 2, 1>& line_end) {
  bool intersects = false;

  Eigen::Matrix<T, 2, 1> line_to_source = ray_source - line_start;
  Eigen::Matrix<T, 2, 1> line_direction = line_end - line_start;
  Eigen::Matrix<T, 2, 1> perpendicular = Perp(ray_direction);

  T denomenator = line_direction.dot(perpendicular);

  T ray_intersect_param = Cross(line_direction, line_to_source) / denomenator;

  T line_intersect_param = line_to_source.dot(perpendicular) / denomenator;

  if (ray_intersect_param >= 0 && line_intersect_param >= 0 &&
      line_intersect_param <= 1) {
    intersects = true;
  }

  return intersects;
}

// Determines whether a ray intersects with a given circle

template <typename T>
bool FurthestFreePointCircle(const Eigen::Matrix<T, 2, 1>& line_start,
                             const Eigen::Matrix<T, 2, 1>& line_end,
                             const Eigen::Matrix<T, 2, 1>& circle_center,
                             const T radius, T* squared_distance,
                             Eigen::Matrix<T, 2, 1>* free_point) {
  bool collides = false;

  if ((line_start - circle_center).squaredNorm() < Sq(radius)) {
    *free_point = line_start;
    collides = true;
  } else {
    Eigen::Matrix<T, 2, 1> projected_point;
    T current_distance;
    ProjectPointOntoLineSegment(circle_center, line_start, line_end,
                                &projected_point, &current_distance);

    if ((projected_point - circle_center).squaredNorm() >= Sq(radius)) {
      *free_point = line_end;
      collides = false;
    } else {
      collides = true;
      Eigen::Matrix<T, 2, 1> translation_vector = projected_point - line_start;
      T slide_back_distance =
          sqrt(Sq(radius) - (projected_point - circle_center).squaredNorm());
      translation_vector -=
          (translation_vector.normalized()) * slide_back_distance;

      *free_point = line_start + translation_vector;
    }
  }

  *squared_distance = ((*free_point) - line_start).squaredNorm();
  return collides;
}

// Returns the Euclidean distance between the two input points
template <typename T>
T EuclideanDistance(const Eigen::Matrix<T, 2, 1>& pos1,
                    const Eigen::Matrix<T, 2, 1>& pos2) {
  if (fabs(pos1.x() - pos2.x()) > 1e-10 || fabs(pos1.y() - pos2.y()) > 1e-10) {
    return (pos2 - pos1).norm();
  } else {
    return 0;
  }
}

template <typename T>
T SquaredDistance(const Eigen::Matrix<T, 2, 1>& pos1,
                  const Eigen::Matrix<T, 2, 1>& pos2) {
  if (fabs(pos1.x() - pos2.x()) > 1e-10 || fabs(pos1.y() - pos2.y()) > 1e-10) {
    return (pos2 - pos1).squaredNorm();
  } else {
    return 0;
  }
}

template <typename T>
T SafeVectorNorm(const Eigen::Matrix<T, 2, 1>& vector) {
  if (fabs(vector.x()) > 1e-10 || fabs(vector.y()) > 1e-10) {
    return vector.norm();
  } else {
    return 0;
  }
}

// Returns the scalar projection of vector1 onto vector2
// vector2 must be nonzero
template <typename T>
T ScalarProjection(const Eigen::Matrix<T, 2, 1>& vector1,
                   const Eigen::Matrix<T, 2, 1>& vector2) {
  return vector1.dot(vector2) / vector2.norm();
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
