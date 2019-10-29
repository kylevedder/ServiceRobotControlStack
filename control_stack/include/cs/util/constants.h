#pragma once

#include <iostream>

#include "config_reader/config_reader.h"

static constexpr bool kProduction = false;
static constexpr float kPi = M_PI;
static constexpr float kEpsilon = 0.001f;

namespace pf {
CONFIG_FLOAT(kLaserStdDev, "pf.kLaserStdDev");
CONFIG_FLOAT(kArcStdDev, "pf.kArcStdDev");
CONFIG_FLOAT(kRotateStdDev, "pf.kRotateStdDev");
CONFIG_FLOAT(kTemporalConsistencyWeight, "pf.kTemporalConsistencyWeight");
}  // namespace pf

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

#define NP_CHECK(exp) \
  if (!kProduction) { \
    CHECK(exp);       \
  }

#define NP_FINITE(exp)         \
  if (!kProduction) {          \
    CHECK(std::isfinite(exp)); \
  }

#define NP_NOT_NAN(exp)      \
  if (!kProduction) {        \
    CHECK(!std::isnan(exp)); \
  }

#define NP_NOT_NULL(exp)     \
  if (!kProduction) {        \
    CHECK((exp) != nullptr); \
  }

#define NP_CHECK_EQ(exp1, exp2) \
  if (!kProduction) {           \
    CHECK((exp1) == (exp2));    \
  }
