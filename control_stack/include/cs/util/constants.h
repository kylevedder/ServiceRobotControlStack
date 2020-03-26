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
#include <iostream>

static constexpr bool kProduction = false;
static constexpr float kEpsilon = 0.001f;
static constexpr float kPi = M_PI;

namespace constants {
static constexpr auto kPositionTopic = "/localization_position";
static constexpr auto kCommandVelocityTopic = "/mobile_base/commands/velocity";
static constexpr auto kOdomTopic = "/odom";
static constexpr auto kLaserTopic = "/scan";
static constexpr auto kGoalTopic = "/nav_goal";
static constexpr auto kTeleopTopic = "/teleop_topic";
}  // namespace constants

static constexpr int kAssertFailReturnCode = 1;

#ifndef CHECK
#define CHECK(exp)                                                      \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" failed!" << std::endl;                             \
    exit(kAssertFailReturnCode);                                        \
  }
#endif

#define CHECK_MSG(exp, msg)                                             \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" failed with message: " << msg << std::endl;        \
    exit(kAssertFailReturnCode);                                        \
  }

#define CHECK_PRINT_VAL(exp, val)                                       \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" (value: " << (val) << ")  failed!" << std::endl;   \
    exit(kAssertFailReturnCode);                                        \
  }

#define CHECK_PRINT_VALS(exp, val1, val2)                               \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" (value: " << (val1) << " vs value: " << (val2)     \
              << ")  failed!" << std::endl;                             \
    exit(kAssertFailReturnCode);                                        \
  }

#define NP_CHECK(exp) \
  if (!kProduction) { \
    CHECK(exp);       \
  }

#define NP_CHECK_MSG(exp, msg) \
  if (!kProduction) {          \
    CHECK_MSG(exp, msg);       \
  }

#define NP_CHECK_VAL(exp, val) \
  if (!kProduction) {          \
    CHECK_PRINT_VAL(exp, val); \
  }

#define NP_CHECK_VALS(exp, val1, val2) \
  if (!kProduction) {                  \
    CHECK_PRINT_VAL(exp, val1, val2);  \
  }

#ifndef CHECK_EQ
#define CHECK_EQ(exp1, exp2) CHECK_PRINT_VALS((exp1) == (exp2), exp1, exp2);
#endif

#define NP_CHECK_EQ(exp1, exp2) \
  if (!kProduction) {           \
    CHECK_EQ(exp1, exp2);       \
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
