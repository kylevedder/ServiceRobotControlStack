#pragma once
// Copyright 2019 kvedder@seas.upenn.edu
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
#include <iostream>

#include "config_reader/macros.h"

static constexpr bool kProduction = false;
static constexpr float kEpsilon = 0.001f;
static constexpr float kPi = M_PI;

namespace constants {
static constexpr float kMinAngle = -kPi / 2;
static constexpr float kMaxAngle = kPi / 2;
static constexpr int kNumReadings = 100;
static constexpr float kAngleDelta =
    std::abs(kMaxAngle - kMinAngle) / static_cast<float>(kNumReadings - 1);
static constexpr float kMinReading = 0.1f;
static constexpr float kMaxReading = 5.0f;

static constexpr auto kCommandVelocityTopic = "/mobile_base/commands/velocity";
static constexpr auto kOdomTopic = "/odom";
static constexpr auto kLaserTopic = "/scan";
}  // namespace constants

#define CHECK(exp)                                                      \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" failed!" << std::endl;                             \
    exit(0);                                                            \
  }

#define CHECK_PRINT_VAL(exp, val)                                       \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" (value: " << (val) << ")  failed!" << std::endl;   \
    exit(0);                                                            \
  }

#define CHECK_PRINT_VALS(exp, val1, val2)                               \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" (value: " << (val1) << " vs value: " << (val2)     \
              << ")  failed!" << std::endl;                             \
    exit(0);                                                            \
  }

#define NP_CHECK(exp) \
  if (!kProduction) { \
    CHECK(exp);       \
  }

#define NP_CHECK_VAL(exp, val) \
  if (!kProduction) {          \
    CHECK_PRINT_VAL(exp, val); \
  }

#define NP_CHECK_VALS(exp, val1, val2) \
  if (!kProduction) {                  \
    CHECK_PRINT_VAL(exp, val1, val2);  \
  }

#define NP_CHECK_EQ(exp1, exp2)                     \
  if (!kProduction) {                               \
    CHECK_PRINT_VALS((exp1) == (exp2), exp1, exp2); \
  }

#define NP_FINITE(exp)                          \
  if (!kProduction) {                           \
    CHECK_PRINT_VAL(std::isfinite(exp), (exp)); \
  }

#define NP_FINITE_2F(exp)                                        \
  if (!kProduction) {                                            \
    CHECK(std::isfinite((exp).x()) && std::isfinite((exp).y())); \
  }

#define NP_NOT_NULL(exp)     \
  if (!kProduction) {        \
    CHECK((exp) != nullptr); \
  }
  