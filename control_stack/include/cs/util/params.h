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

namespace params {
static constexpr float kMinDistanceThreshold = 0.05;
static constexpr float kProposedTranslationStdDev = 0.3;
static constexpr float kProposedRotationStdDev = 5;

static constexpr float kMaxTraAcc = 3;
static constexpr float kMaxTraVel = 1;
static constexpr float kMaxRotAcc = 2;
static constexpr float kMaxRotVel = 1;

static constexpr float kOdomFilteringPriorBias = 0.7;
static constexpr float kThresholdRotateInPlace = 0.9;
static constexpr float kTranslationCostScaleFactor = 1000;
}  // namespace params