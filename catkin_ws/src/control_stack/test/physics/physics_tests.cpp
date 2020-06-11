// Copyright 2020 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
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

#include <gtest/gtest.h>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "cs/util/physics.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

using util::physics::ApplyCommandLimits;

TEST(ApplyCommandLimits, ZeroZero) {
  const util::Twist cmd(0, 0, 0);
  const util::Twist limited_cmd =
      ApplyCommandLimits(cmd, 1, {0, 0, 0}, 1.0f, 1.0f, 1.0f, 1.0f);
  EXPECT_EQ(limited_cmd, cmd);
}
