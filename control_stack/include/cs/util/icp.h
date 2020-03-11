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
#include <limits>
#include <utility>
#include <vector>

#include "ceres/ceres.h"
#include "cs/util/laser_scan.h"

namespace util {
namespace solver {

// using ceres::AutoDiffCostFunction;
// using ceres::CostFunction;
// using ceres::Problem;
// using ceres::Solve;
// using ceres::Solver;

struct CostFunctor {
  const std::vector<Eigen::Vector2f> target_points_;
  const std::vector<Eigen::Vector2f> current_points_;

  CostFunctor() = delete;

  CostFunctor(const std::vector<Eigen::Vector2f>& target_points,
              const std::vector<Eigen::Vector2f>& current_points)
      : target_points_(target_points), current_points_(current_points) {
    for (const auto& p : target_points_) {
      CHECK(std::isfinite(p.x()));
      CHECK(std::isfinite(p.y()));
    }
    for (const auto& p : current_points_) {
      CHECK(std::isfinite(p.x()));
      CHECK(std::isfinite(p.y()));
    }
  }

  template <typename T>
  bool operator()(const T* const x_ptr,
                  const T* const y_ptr,
                  const T* const rotation_ptr,
                  T* residual) const {
    static const T max_error = T(std::numeric_limits<double>::max());
    if (ceres::IsInfinite(*x_ptr) || ceres::IsInfinite(*y_ptr) ||
        ceres::IsInfinite(*rotation_ptr)) {
      return false;
    }
    auto transform = Eigen::Transform<T, 2, Eigen::Affine>::Identity();
    transform.translate(Eigen::Matrix<T, 2, 1>(*x_ptr, *y_ptr));
    transform.rotate(*rotation_ptr);

    (*residual) = T(0);
    for (const auto& cp : current_points_) {
      const Eigen::Matrix<T, 2, 1> current = transform * cp.template cast<T>();
      T lowest_error = max_error;
      for (const auto& tp : target_points_) {
        const T dist = (current - tp.template cast<T>()).squaredNorm();
        lowest_error = std::min(lowest_error, dist);
      }
      if (lowest_error < max_error) {
        (*residual) += lowest_error;
      }
    }
    return true;
  }
};

std::pair<bool, util::Twist> ICP(const util::LaserScan& scan_0,
                                 const util::LaserScan& scan_1,
                                 const util::Twist& est_transform) {
  double delta_x = static_cast<double>(est_transform.tra.x());
  double delta_y = static_cast<double>(est_transform.tra.y());
  double delta_rot = static_cast<double>(est_transform.rot);

  // Build the problem.
  ceres::Problem problem;
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1>(new CostFunctor(
          scan_1.TransformPointsFrameSparse(Eigen::Affine2f::Identity()),
          scan_0.TransformPointsFrameSparse(Eigen::Affine2f::Identity())));
  problem.AddResidualBlock(
      cost_function, nullptr, &delta_x, &delta_y, &delta_rot);
  // Run the solver!
  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  //  std::cout << summary.BriefReport() << "\n";
  return {summary.termination_type == ceres::TerminationType::CONVERGENCE,
          util::Twist(delta_x, delta_y, delta_rot)};
}

}  // namespace solver
}  // namespace util
