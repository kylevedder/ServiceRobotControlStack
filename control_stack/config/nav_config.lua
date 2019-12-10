-- Copyright 2019 kvedder@seas.upenn.edu
-- School of Engineering and Applied Sciences,
-- University of Pennsylvania
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
-- SOFTWARE.
-- ========================================================================

pf = {
  kLaserStdDev = 0.1;
  kArcStdDev = 0.1;
  kRotateStdDev = 0.04;
  kTemporalConsistencyWeight = 0;

  kMap = "./src/ServiceRobotControlStack/control_stack/maps/loop.map";
  kInitX = 4;
  kInitY = 0;
  kInitTheta = 0;
  kRobotRadius = 0.1;
  kSafetyMargin = 0.1;
  kCollisionRollout = 2;
};

od = {
  kMinDistanceThreshold = 0.05;
  kProposedTranslationStdDev = 1.0;
  kProposedRotationStdDev = 5;
  kDesiredCommandX = 0.2;
  kDesiredCommandRot = 0;
  kOdomFilteringPriorBias = 0.7;
  kThresholdRotateInPlace = 0.9;
  kTranslateCommandSign = 1;
  kOdomFilteringPriorBias = 0.7;
  kThresholdRotateInPlace = 0.9;
  kTranslationCostScaleFactor = 1000;
};

limits = {
  kMaxTraAcc = 3;
  kMaxTraVel = 1;
  kMaxRotAcc = 2;
  kMaxRotVel = 1;
};

path_finding = {
  switch_historesis_threshold = 0.4; -- Meters
};

rrt = {
  max_iterations = 1000;
  goal_bias = 0.1;
  is_goal_threshold = 0.4; -- Meters
  delta_q = 1; -- Meters
};

control = {
  rotation_drive_threshold = 0.2; -- Radians.
  rotation_p = 1;
  translation_p = 1;
};