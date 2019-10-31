#pragma once

#include <iostream>

static constexpr bool kProduction = false;
static constexpr float kPi = M_PI;
static constexpr float kEpsilon = 0.001f;

static constexpr float kRobotMaxAccel = 0.1;   // m/s^2.
static constexpr float kRobotMaxVelocity = 1;  // m/s.

static constexpr float kMinAngle = -kPi / 2;
static constexpr float kMaxAngle = kPi / 2;
static constexpr int kNumReadings = 100;
static constexpr float kAngleDelta =
    std::abs(kMaxAngle - kMinAngle) / static_cast<float>(kNumReadings - 1);
static constexpr float kMinReading = 0.1f;
static constexpr float kMaxReading = 5.0f;

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

#define NP_CHECK(exp) \
  if (!kProduction) { \
    CHECK(exp);       \
  }

#define NP_CHECK_VAL(exp, val) \
  if (!kProduction) {          \
    CHECK_PRINT_VAL(exp, val); \
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

#define NP_CHECK_EQ(exp1, exp2) \
  if (!kProduction) {           \
    CHECK((exp1) == (exp2));    \
  }
