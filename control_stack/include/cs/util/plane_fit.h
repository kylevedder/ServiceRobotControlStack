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

#include <algorithm>
#include <array>
#include <random>
#include <utility>
#include <vector>

#include "cs/util/point_cloud.h"
#include "shared/math/geometry.h"

namespace util {

struct Plane {
  Eigen::Vector3f anchor;
  Eigen::Vector3f v1;
  Eigen::Vector3f v1_normed;
  Eigen::Vector3f v2;
  Eigen::Vector3f v2_normed;

  Plane()
      : anchor(Eigen::Vector3f::Zero()),
        v1(Eigen::Vector3f::Zero()),
        v1_normed(Eigen::Vector3f::Zero()),
        v2(Eigen::Vector3f::Zero()),
        v2_normed(Eigen::Vector3f::Zero()) {}

  Plane(const Eigen::Vector3f& p1,
        const Eigen::Vector3f& p2,
        const Eigen::Vector3f& p3)
      : anchor(p1),
        v1(p2 - anchor),
        v1_normed(geometry::GetNormalizedOrZero(v1)),
        // Ensures that v1 and v2 are orthogonal.
        v2((p3 - anchor) - (p3 - anchor).dot(v1_normed) * v1_normed),
        v2_normed(geometry::GetNormalizedOrZero(v2)) {}

  Plane(const Eigen::Vector3f& center,
        const Eigen::Vector3f& v1_normed,
        const float v1_max,
        const Eigen::Vector3f& v2_normed,
        const float v2_max)
      : anchor(center - v1_normed * v1_max - v2_normed * v2_max),
        v1(v1_normed * v1_max * 2),
        v1_normed(v1_normed),
        v2(v2_normed * v2_max * 2),
        v2_normed(v2_normed) {}

  Eigen::Vector3f ProjectOntoPlane(const Eigen::Vector3f& q) const {
    const Eigen::Vector3f diff = q - anchor;
    const Eigen::Vector3f v1_proj = diff.dot(v1_normed) * v1_normed;
    const Eigen::Vector3f v2_proj = diff.dot(v2_normed) * v2_normed;
    return anchor + v1_proj + v2_proj;
  }

  float SquaredDistanceFromPlane(const Eigen::Vector3f& q) const {
    const Eigen::Vector3f diff = q - anchor;
    const Eigen::Vector3f v1_proj = diff.dot(v1_normed) * v1_normed;
    const Eigen::Vector3f v2_proj = diff.dot(v2_normed) * v2_normed;
    const Eigen::Vector3f plane_projection = anchor + v1_proj + v2_proj;
    const float off_manifold_distance = (plane_projection - q).squaredNorm();
    const float off_square_distance =
        (v1_proj - v1).squaredNorm() + (v2_proj - v2).squaredNorm();
    return off_square_distance + off_manifold_distance;
  }

  Eigen::Quaternionf ToQuaternion() const {
    const auto plane_norm = v1_normed.cross(v2_normed).normalized();
    Eigen::Matrix3f mat = Eigen::Matrix3f::Identity();
    mat.col(0) = v1_normed;
    mat.col(1) = v2_normed;
    mat.col(2) = plane_norm;
    return Eigen::Quaternionf(mat);
  }
};

namespace pca {
template <typename PC>
Plane FitPlane(const PC& pc) {
  std::vector<Eigen::Vector3f> points;
  for (const auto& p : pc) {
    if (!p.IsValid()) {
      continue;
    }
    points.push_back(p.GetMappedVector3f());
  }

  if (points.empty()) {
    return {};
  }

  Eigen::Matrix<float, 3, Eigen::Dynamic> data_matrix(3, points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    data_matrix.col(i) = points[i];
  }
  // Subtract the mean.
  const Eigen::Vector3f center = data_matrix.rowwise().mean();
  data_matrix = data_matrix.colwise() - center;
  const Eigen::Matrix3f covariance_matrix =
      data_matrix * data_matrix.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(covariance_matrix);
  const auto eigen_vectors = es.eigenvectors();
  // Eigenvectors of the largest two eigenvalues. Normalized by the solver.
  const Eigen::Vector3f v1 = eigen_vectors.col(1);
  const Eigen::Vector3f v2 = eigen_vectors.col(2);

  float v1_dist = 0;
  float v2_dist = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    v1_dist = std::max(std::abs(data_matrix.col(i).dot(v1)), v1_dist);
    v2_dist = std::max(std::abs(data_matrix.col(i).dot(v2)), v2_dist);
  }

  return Plane(center, v1, v1_dist, v2, v2_dist);
}
}  // namespace pca

namespace ransac {
template <typename PC, int Iterations>
Plane FitPlane(const PC& pc) {
  static std::default_random_engine generator;
  std::vector<Eigen::Vector3f> points;
  for (const auto& p : pc) {
    if (!p.IsValid()) {
      continue;
    }
    points.push_back(p.GetMappedVector3f());
  }
  std::uniform_int_distribution<int> distribution(0, points.size());

  std::array<std::pair<Plane, float>, Iterations> plane_arr;
  for (int i = 0; i < Iterations; ++i) {
    const Eigen::Vector3f& p1 = points[distribution(generator)];
    const Eigen::Vector3f& p2 = points[distribution(generator)];
    const Eigen::Vector3f& p3 = points[distribution(generator)];
    plane_arr[i].first = {p1, p2, p3};
    plane_arr[i].second = 0;

    for (const auto& p : points) {
      plane_arr[i].second += plane_arr[i].first.SquaredDistanceFromPlane(p);
    }
  }

  size_t min_index = 0;
  for (size_t i = 0; i < plane_arr.size(); ++i) {
    if (plane_arr[i].second < plane_arr[i].second) {
      min_index = i;
    }
  }

  return plane_arr[min_index].first;
}

}  // namespace ransac
}  // namespace util
