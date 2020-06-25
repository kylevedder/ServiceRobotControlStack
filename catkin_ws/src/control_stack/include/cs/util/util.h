#pragma once
// Copyright 2019 kvedder@seas.upenn.edu
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

#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <unistd.h>
#include <iostream>
#include <string>

namespace util {

inline void PrintCurrentWorkingDirectory() {
  char cwd[1024 * 1024] = {0};
  if (getcwd(cwd, sizeof(cwd)) != nullptr) {
    std::cout << "Current working dir: " << cwd << std::endl;
  } else {
    std::cerr << "Error calling getcwd()" << std::endl;
  }
}

inline std::string GetCurrentWorkingDirectory() {
  char cwd[1024 * 1024] = {0};
  if (getcwd(cwd, sizeof(cwd)) != nullptr) {
    return std::string(cwd);
  }
  return "Error calling getcwd()";
}

inline tf::Transform EigenAffineToTFTransform(const Eigen::Affine3f& aff) {
  tf::Transform tf = tf::Transform::getIdentity();
  tf::transformEigenToTF(aff.cast<double>(), tf);
  return tf;
}

}  // namespace util
