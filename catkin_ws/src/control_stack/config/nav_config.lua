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
  deadzones = {0, 85, 640, 725};
  center_offset = {0.2, 0};
};

state_estimation = {
  use_sim_ground_truth = false;
};

function get_goals()
  return {{35, -40, 0}, {-51, 3, -3.14}, {17, -14, -3.14}}
end

pf = {
  kLaserStdDev = 0.1;
  kArcStdDev = 0.07;
  kRotateStdDev = 0.06;
  kTemporalConsistencyWeight = 0;

  -- kMap = "../rosbuild_ws/simulator/f1tenth_simulator/maps/GDC3.txt";
  map = "src/control_stack/maps/outside_grasp.map";
  map = "src/control_stack/maps/fourthfloorloop.map";
  goal_poses = get_goals();
  start_pose = get_goals()[3];

  kRobotRadius = 0.1;
  kSafetyMargin = 0.16;
  kCollisionRollout = 2;
};

frames = {
  laser_tf_frame = "/laser";
  base_tf_frame = "/base_link";
  map_tf_frame = "/map";
};

od = {
  kMinDistanceThreshold = 0.05;
  kDistanceFromMax = 0.1;
  kProposedTranslationStdDev = 1.0;
  kProposedRotationStdDev = 5;
  command_bias = 0.9;
  kThresholdRotateInPlace = 0.9;
  kTranslationCostScaleFactor = 1000;
  clustering = {
    max_dist_between_readings = 0.05;
    min_distance_btw_readings_to_reason_angle = 0.01;
    line_similarity = math.cos(math.rad(20));
  };
  is_wall_threshold = 0.1;
  min_trajectory_rotation = 0.005;
};

limits = {
  kMaxTraAcc = 0.2;
  kMaxTraVel = 1;
  kMaxRotAcc = 2.5;
  kMaxRotVel = 1;
};

safety = {
  decelerate_scaler = 0.75;
};

path_finding = {
  goal_delta_change = 0.8; -- Meters
  switch_historesis_threshold = 0.4; -- Meters
  max_distance_off_path = 0.2; -- Meters
  local_robot_inflation = 1.1;
  global_robot_inflation = 1.5;
};

rrt = {
  max_iterations = 1000;
  goal_bias = 0.1;
  is_goal_threshold = 0.4; -- Meters
  delta_q = 0.25; -- Meters
};

control = {
  rotation_drive_threshold = 0.4; -- Radians.
  rotation_p = 0.95;
  rotation_i = 0.0;
  translation_p = 0.5;
  stop_past_goal_threshold = 0.75;
  stop_past_goal_dampener = 5;
  goal_deadzone_tra = 0.3; -- Meters.
  goal_deadzone_rot = 0.2; -- Radians.
};

cmd_scaler = {
  command_scaler = "turtlebot";
  rotation_zero_threshold = 0.01;
  rotation_min_effect_threshold = 0.4;
  rotation_translation_scaler = 1.9;
  rotation_translation_exponent = 2.8;
};

esc_collision = {
  num_safety_margins = 2;
};
