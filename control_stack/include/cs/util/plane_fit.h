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
  Eigen::Vector3f center;
  Eigen::Vector3f v1;
  Eigen::Vector3f v1_normed;
  Eigen::Vector3f v2;
  Eigen::Vector3f v2_normed;

  Plane()
      : center(Eigen::Vector3f::Zero()),
        v1(Eigen::Vector3f::Zero()),
        v1_normed(Eigen::Vector3f::Zero()),
        v2(Eigen::Vector3f::Zero()),
        v2_normed(Eigen::Vector3f::Zero()) {}

  Plane(const Eigen::Vector3f& center,
        const Eigen::Vector3f& v1_normed,
        const float v1_magnitude,
        const Eigen::Vector3f& v2_normed,
        const float v2_magnitude)
      : center(center),
        v1(v1_normed * v1_magnitude),
        v1_normed(v1_normed),
        v2(v2_normed * v2_magnitude),
        v2_normed(v2_normed) {}

  Eigen::Quaternionf ToQuaternion() const {
    const auto plane_norm = v1_normed.cross(v2_normed).normalized();
    Eigen::Matrix3f mat = Eigen::Matrix3f::Identity();
    mat.col(0) = v1_normed;
    mat.col(1) = v2_normed;
    mat.col(2) = plane_norm;
    return Eigen::Quaternionf(mat);
  }

  struct Corners {
    Eigen::Vector3f upper_left;
    Eigen::Vector3f upper_right;
    Eigen::Vector3f lower_left;
    Eigen::Vector3f lower_right;

    Corners()
        : upper_left(Eigen::Vector3f::Zero()),
          upper_right(Eigen::Vector3f::Zero()),
          lower_left(Eigen::Vector3f::Zero()),
          lower_right(Eigen::Vector3f::Zero()) {}
  };

  Corners ToCorners() const {
    static const Eigen::Vector3f kVerticalVector(0, 0, 1);
    static const Eigen::Vector3f kHorizontalVector(0, 1, 0);

    const float v1_vert_dist = kVerticalVector.dot(v1_normed);
    const float v2_vert_dist = kVerticalVector.dot(v2_normed);

    const float v1_horiz_dist = kHorizontalVector.dot(v1_normed);
    const float v2_horiz_dist = kHorizontalVector.dot(v2_normed);

    // If v1 is vertical, its dot product will have a larger magnitude.
    const bool v1_vert = (std::abs(v1_vert_dist) > std::abs(v2_vert_dist));

    // Sign() ensures that the vert vector is pointing positive Z.
    Eigen::Vector3f vert = ((v1_vert) ? (v1 * math_util::Sign(v1_vert_dist))
                                      : (v2 * math_util::Sign(v2_vert_dist)));
    // Sign() ensures that the horiz vector is pointing positive Y.
    Eigen::Vector3f horiz = ((v1_vert) ? (v2 * math_util::Sign(v2_horiz_dist))
                                       : (v1 * math_util::Sign(v1_horiz_dist)));

    Corners corners;
    corners.upper_left = center + horiz + vert;
    corners.upper_right = center - horiz + vert;
    corners.lower_left = center + horiz - vert;
    corners.lower_right = center - horiz - vert;
    return corners;
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
}  // namespace util
