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
using util::physics::CommandDelta;
using util::physics::ComputeCommandDelta;
using util::physics::ComputeFullStop;
using util::physics::StopDelta;

// clang-format off
TEST(ApplyCommandLimits, ZeroZero) {
  const util::Twist cmd(0, 0, 0);
  const util::Twist limited_cmd =
      ApplyCommandLimits(cmd, 1, {0, 0, 0}, 1.0f, 1.0f, 1.0f, 1.0f);
  EXPECT_EQ(limited_cmd, cmd);
}

TEST(ApplyCommandLimits, OverMaxAccNotVel) {
  const util::Twist cmd(2, 0, 0);
  const util::Twist expected_cmd(0.2, 0, 0);
  const util::Twist limited_cmd =
      ApplyCommandLimits(cmd,
                         0.1,
                         {0, 0, 0},
                         1.0f,
                         2.0f,
                         1.0f,
                         1.0f);
  EXPECT_EQ(limited_cmd, expected_cmd);
}

TEST(ApplyCommandLimits, OverMaxVelNotAcc) {
  const util::Twist cmd(2, 0, 0);
  const util::Twist expected_cmd(0.1, 0, 0);
  const util::Twist limited_cmd =
      ApplyCommandLimits(cmd,
                         0.1,
                         {0, 0, 0},
                         0.1f,
                         2.0f,
                         1.0f,
                         1.0f);
  EXPECT_EQ(limited_cmd, expected_cmd);
}

TEST(ApplyCommandLimits, NegOverMaxAccNotVel) {
  const util::Twist cmd(-2, 0, 0);
  const util::Twist expected_cmd(-0.2, 0, 0);
  const util::Twist limited_cmd =
      ApplyCommandLimits(cmd,
                         0.1,
                         {0, 0, 0},
                         1.0f,
                         2.0f,
                         1.0f,
                         1.0f);
  EXPECT_EQ(limited_cmd, expected_cmd);
}

TEST(ApplyCommandLimits, NegOverMaxVelNotAcc) {
  const util::Twist cmd(-2, 0, 0);
  const util::Twist expected_cmd(-0.1, 0, 0);
  const util::Twist limited_cmd =
      ApplyCommandLimits(cmd,
                         0.1,
                         {0, 0, 0},
                         0.1f,
                         2.0f,
                         1.0f,
                         1.0f);
  EXPECT_EQ(limited_cmd, expected_cmd);
}

TEST(ComputeCommandDelta, NoMove) {
  const util::Twist current_velocity(0, 0, 0);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = 0.5;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, current_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), current_velocity);
  EXPECT_EQ(command_delta.GetEndPosition(), util::Pose(0, 0, 0));
}

TEST(ComputeCommandDelta, MoveConstant) {
  const util::Twist current_velocity(1, 0, 0);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = 0.5;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, current_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), current_velocity);
  EXPECT_EQ(command_delta.GetEndPosition(), util::Pose(0.5, 0, 0));
}

TEST(ComputeCommandDelta, MoveConstantBackwards) {
  const util::Twist current_velocity(-1, 0, 0);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = 0.5;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, current_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), current_velocity);
  EXPECT_EQ(command_delta.GetEndPosition(), util::Pose(-0.5, 0, 0));
}

TEST(ComputeCommandDelta, MoveConstantTurnLeft180) {
  const util::Twist current_velocity(1, 0, 1);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = M_PI;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, current_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), current_velocity);
  EXPECT_NEAR(command_delta.GetEndPosition().tra.x(), 0, 0.0001);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().tra.y(), 2);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().rot, M_PI);
}

TEST(ComputeCommandDelta, MoveConstantTurnRight180) {
  const util::Twist current_velocity(1, 0, -1);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = M_PI;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, current_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), current_velocity);
  EXPECT_NEAR(command_delta.GetEndPosition().tra.x(), 0, 0.0001);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().tra.y(), -2);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().rot, -M_PI);
}

TEST(ComputeCommandDelta, MoveConstantTurnLeft90) {
  const util::Twist current_velocity(1, 0, 0.5);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = M_PI;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, current_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), current_velocity);
  EXPECT_NEAR(command_delta.GetEndPosition().tra.x(), 2, 0.0001);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().tra.y(), 2);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().rot, M_PI / 2);
}

TEST(ComputeCommandDelta, MoveConstantStraightTurnLeft90) {
  const util::Twist current_velocity(1, 0, 0);
  const util::Twist commanded_velocity(1, 0, 0.5);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = M_PI;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, commanded_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), commanded_velocity);
  EXPECT_NEAR(command_delta.GetEndPosition().tra.x(), 2, 0.0001);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().tra.y(), 2);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().rot, M_PI / 2);
}

TEST(ComputeCommandDelta, MoveConstantTurnRight90) {
  const util::Twist current_velocity(1, 0, -0.5);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = M_PI;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, current_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), current_velocity);
  EXPECT_NEAR(command_delta.GetEndPosition().tra.x(), 2, 0.0001);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().tra.y(), -2);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().rot, -M_PI / 2);
}

TEST(ComputeCommandDelta, MoveConstantStraightTurnRight90) {
  const util::Twist current_velocity(1, 0, 0);
  const util::Twist commanded_velocity(1, 0, -0.5);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = M_PI;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, commanded_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), commanded_velocity);
  EXPECT_NEAR(command_delta.GetEndPosition().tra.x(), 2, 0.0001);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().tra.y(), -2);
  EXPECT_FLOAT_EQ(command_delta.GetEndPosition().rot, -M_PI / 2);
}

TEST(ComputeCommandDelta, MoveAccel) {
  const util::Twist current_velocity(1, 0, 0);
  const util::Twist command_velocity(2, 0, 0);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = 0.5;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, command_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), command_velocity);

  // dx = vi dt * 0.5 a t^2
  // dx = vi dt * 0.5 (a * t) t
  // dx = 1 * 0.5 + 0.5 * 1 * 0.5
  // dx = 0.75
  EXPECT_EQ(command_delta.GetEndPosition(), util::Pose(0.75, 0, 0));
}

TEST(ComputeCommandDelta, MoveDecel) {
  const util::Twist current_velocity(1, 0, 0);
  const util::Twist command_velocity(-1, 0, 0);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = 0.5;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, command_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), command_velocity);

  // dx = vi dt * 0.5 a t^2
  // dx = vi dt * 0.5 (a * t) t
  // dx = 1 * 0.5 + 0.5 * -2 * 0.5
  // dx = 0
  EXPECT_EQ(command_delta.GetEndPosition(), util::Pose(0, 0, 0));
}

TEST(ComputeFullStop, MoveConstantDecel) {
  const util::Twist current_velocity(1, 0, 0);
  const util::Twist command_velocity(2, 0, 0);
  const util::Pose current_pose(0, 0, 0);
  const float command_time_delta = 0.5;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, command_velocity, command_time_delta);

  // dx = vi dt * 0.5 a t^2
  // dx = vi dt * 0.5 (a * t) t
  // dx = 1 * 0.5 + 0.5 * 1 * 0.5
  // dx = 0.75
  ASSERT_EQ(command_delta.GetEndPosition(), util::Pose(0.75, 0, 0));
  ASSERT_EQ(command_delta.GetEndVelocity(), command_velocity);

  const float decel_cap = 0.5;
  StopDelta stop_delta = ComputeFullStop(command_delta, decel_cap);
  EXPECT_EQ(stop_delta.current_position_wf, command_delta.GetEndPosition());

  // vf^2 = vi^2 + 2 a dx
  // 0 - vi^2 =  + 2 a dx
  // (-vi^2) / (2 a) =  dx
  // (2^2) / (2 * 0.5) =  dx
  // 4 =  dx
  EXPECT_EQ(stop_delta.stop_position_wf, util::Pose(4.75, 0, 0));
}

TEST(ComputeFullStop, MoveReverseDecel) {
  const util::Twist current_velocity(1, 0, 0);
  const util::Twist command_velocity(-1, 0, 0);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = 0.5;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, command_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), command_velocity);

  // dx = vi dt * 0.5 a t^2
  // dx = vi dt * 0.5 (a * t) t
  // dx = 1 * 0.5 + 0.5 * -2 * 0.5
  // dx = 0
  ASSERT_EQ(command_delta.GetEndPosition(), util::Pose(0, 0, 0));
  ASSERT_EQ(command_delta.GetEndVelocity(), command_velocity);

  const float decel_cap = 0.5;
  StopDelta stop_delta = ComputeFullStop(command_delta, decel_cap);
  EXPECT_EQ(stop_delta.current_position_wf, command_delta.GetEndPosition());

  // vf^2 = vi^2 + 2 a dx
  // 0 - vi^2 =  + 2 a dx
  // (-vi^2) / (2 a) =  dx
  // (1^2) / (2 * 0.5) =  dx
  // 1 =  dx
  EXPECT_EQ(stop_delta.stop_position_wf, util::Pose(-1, 0, 0));
}

TEST(ComputeFullStop, MoveReverseDecel2) {
  const util::Twist current_velocity(1, 0, 0);
  const util::Twist command_velocity(-1, 0, 0);
  const util::Pose current_pose(0, 0, 0);
  const float time_delta = 0.5;
  const CommandDelta command_delta = ComputeCommandDelta(current_pose, current_velocity, command_velocity, time_delta);
  EXPECT_EQ(command_delta.GetEndVelocity(), command_velocity);

  // dx = vi dt * 0.5 a t^2
  // dx = vi dt * 0.5 (a * t) t
  // dx = 1 * 0.5 + 0.5 * -2 * 0.5
  // dx = 0
  ASSERT_EQ(command_delta.GetEndPosition(), util::Pose(0, 0, 0));
  ASSERT_EQ(command_delta.GetEndVelocity(), command_velocity);

  const float decel_cap = 0.25;
  StopDelta stop_delta = ComputeFullStop(command_delta, decel_cap);
  EXPECT_EQ(stop_delta.current_position_wf, command_delta.GetEndPosition());

  // vf^2 = vi^2 + 2 a dx
  // 0 - vi^2 =  + 2 a dx
  // (-vi^2) / (2 a) =  dx
  // (1^2) / (2 * 0.25) =  dx
  // 2 =  dx
  EXPECT_EQ(stop_delta.stop_position_wf, util::Pose(-2, 0, 0));
}

// clang-format on