-- Copyright 2019 - 2020 kvedder@seas.upenn.edu
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

laser = {
  laser_deadzone_left_min = 640;
  laser_deadzone_left_max = 725;
  laser_deadzone_right_min = 0;
  laser_deadzone_right_max = 85;
};

pf = {
  kLaserStdDev = 0.1;
  kArcStdDev = 0.05;
  kRotateStdDev = 0.04;
  kTemporalConsistencyWeight = 0;

  kMap = "./src/ServiceRobotControlStack/control_stack/maps/outside_grasp.map";
  kInitX = -1;
  kInitY = -1.95;
  kInitTheta = 0;
  kGoalX = -1;
  kGoalY = -7.95;
  kRobotRadius = 0.1;
  kSafetyMargin = 0.2;
  kCollisionRollout = 2;
};

od = {
  kMinDistanceThreshold = 0.05;
  kDistanceFromMax = 0.1;
  kProposedTranslationStdDev = 1.0;
  kProposedRotationStdDev = 5;
  kOdomFilteringPriorBias = 0.9;
  kThresholdRotateInPlace = 0.9;
  kTranslateCommandSign = 1;
  kTranslationCostScaleFactor = 1000;
  clustering = {
    max_dist_between_readings = 0.05;
    min_distance_btw_readings_to_reason_angle = 0.01;
    line_similarity = math.cos(math.rad(20));
  };
  min_trajectory_rotation = 0.005;
};

limits = {
  kMaxTraAcc = 0.2;
  kMaxTraVel = 1;
  kMaxRotAcc = 2;
  kMaxRotVel = 1;
};

path_finding = {
  goal_delta_change = 0.8; -- Meters
  switch_historesis_threshold = 0.4; -- Meters
  max_distance_off_path = 0.2; -- Meters
};

rrt = {
  max_iterations = 10000;
  goal_bias = 0.1;
  is_goal_threshold = 0.4; -- Meters
  delta_q = 0.25; -- Meters
};

control = {
  rotation_drive_threshold = 0.7; -- Radians.
  rotation_p = 0.75;
  translation_p = 0.5;
  goal_deadzone_tra = 0.1; -- Meters.
};