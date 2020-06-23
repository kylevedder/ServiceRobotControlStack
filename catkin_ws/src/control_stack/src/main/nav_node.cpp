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

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <fstream>
#include <string>

#include "config_reader/config_reader.h"
#include "cs/localization/particle_filter.h"
#include "cs/motion_planning/pid.h"
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/path_finding/astar.h"
#include "cs/path_finding/global_path_finder.h"
#include "cs/state_estimation/pf_state_estimator.h"
#include "cs/state_estimation/sim_state_estimator.h"
#include "cs/state_estimation/state_estimator.h"
#include "cs/util/physics.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

static constexpr bool kDebug = true;

namespace params {
CONFIG_STRING(map, "pf.map");
CONFIG_VECTOR3F(start_pose, "pf.start_pose");
CONFIG_VECTOR3FLIST(goal_poses, "pf.goal_poses");
CONFIG_FLOAT(kRobotRadius, "pf.kRobotRadius");
CONFIG_FLOAT(kSafetyMargin, "pf.kSafetyMargin");

CONFIG_FLOAT(kMaxTraAcc, "limits.kMaxTraAcc");
CONFIG_FLOAT(kMaxTraVel, "limits.kMaxTraVel");
CONFIG_FLOAT(kMaxRotAcc, "limits.kMaxRotAcc");
CONFIG_FLOAT(kMaxRotVel, "limits.kMaxRotVel");

CONFIG_STRING(map_tf_frame, "frames.map_tf_frame");
CONFIG_STRING(base_link_tf_frame, "frames.base_tf_frame");
CONFIG_STRING(laser_tf_frame, "frames.laser_tf_frame");

CONFIG_INTLIST(deadzones, "laser.deadzones");
CONFIG_BOOL(use_sim_ground_truth, "state_estimation.use_sim_ground_truth");
}  // namespace params

struct CallbackWrapper {
  util::vector_map::VectorMap map_;
  std::unique_ptr<cs::state_estimation::StateEstimator> state_estimator_;
  cs::obstacle_avoidance::ObstacleDetector obstacle_detector_;
  cs::motion_planning::PIDController motion_planner_;
  cs::path_finding::GlobalPathFinder<cs::path_finding::AStar<3, 20000, true>>
      global_path_finder_;
  cs::path_finding::AStar<5, 500, false> local_path_finder_;
  tf::TransformBroadcaster br_;
  util::Pose current_goal_;
  int current_goal_index_;

  // Functionality pub/sub
  ros::Publisher position_pub_;
  ros::Publisher command_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber teleop_sub_;

  // Debug pub/sub
  ros::Publisher modified_laser_pub_;
  ros::Publisher particle_pub_;
  ros::Publisher map_pub_;
  ros::Publisher detected_walls_pub_;
  ros::Publisher robot_size_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher robot_path_pub_;
  ros::Publisher base_link_robot_path_pub_;
  ros::Publisher rrt_tree_pub_;

  CallbackWrapper() = delete;

  cs::state_estimation::StateEstimator* MakeStateEstimator(ros::NodeHandle* n) {
    if (params::CONFIG_use_sim_ground_truth) {
      ROS_INFO("Using sim ground truth for state estimation");
      return new cs::state_estimation::SimStateEstimator(n);
    }
    ROS_INFO("Using PF for state estimation initialized at (%f, %f), %f",
             params::CONFIG_start_pose.x(),
             params::CONFIG_start_pose.y(),
             params::CONFIG_start_pose.z());
    return new cs::state_estimation::PFStateEstimator(
        map_, util::Pose(params::CONFIG_start_pose));
  }

  CallbackWrapper(const std::string& map_file, ros::NodeHandle* n)
      : map_(map_file),
        state_estimator_(MakeStateEstimator(n)),
        obstacle_detector_(map_),
        motion_planner_(map_, *state_estimator_),
        global_path_finder_(
            map_, params::CONFIG_kRobotRadius, params::CONFIG_kSafetyMargin),
        local_path_finder_(
            map_, params::CONFIG_kRobotRadius, params::CONFIG_kSafetyMargin),
        current_goal_(util::Pose(params::CONFIG_goal_poses.front())),
        current_goal_index_(0) {
    position_pub_ =
        n->advertise<geometry_msgs::Twist>(constants::kPositionTopic, 1);
    command_pub_ =
        n->advertise<geometry_msgs::Twist>(constants::kCommandVelocityTopic, 1);
    laser_sub_ = n->subscribe(
        constants::kLaserTopic, 1, &CallbackWrapper::LaserCallback, this);
    odom_sub_ = n->subscribe(
        constants::kOdomTopic, 1, &CallbackWrapper::OdomCallback, this);
    teleop_sub_ = n->subscribe(
        constants::kGoalTopic, 10, &CallbackWrapper::GoalCallback, this);

    if (kDebug) {
      modified_laser_pub_ =
          n->advertise<sensor_msgs::LaserScan>("scan_modified", 10);
      particle_pub_ =
          n->advertise<visualization_msgs::MarkerArray>("particles", 10);
      map_pub_ = n->advertise<visualization_msgs::Marker>("robot_map", 10);
      detected_walls_pub_ = n->advertise<visualization_msgs::MarkerArray>(
          "detected_obstacles", 10);
      robot_size_pub_ =
          n->advertise<visualization_msgs::Marker>("robot_size", 10);
      goal_pub_ = n->advertise<visualization_msgs::MarkerArray>("goal", 10);
      robot_path_pub_ =
          n->advertise<visualization_msgs::Marker>("robot_path", 10);
      base_link_robot_path_pub_ =
          n->advertise<visualization_msgs::Marker>("base_link_robot_path", 10);
      rrt_tree_pub_ = n->advertise<visualization_msgs::Marker>("rrt_tree", 10);
    }
  }

  void GoalCallback(const geometry_msgs::Pose2D& msg) {
    current_goal_ = {static_cast<float>(msg.x),
                     static_cast<float>(msg.y),
                     static_cast<float>(msg.theta)};
  }

  void CleanLaserScan(util::LaserScan* laser) {
    NP_CHECK(params::CONFIG_deadzones.size() % 2 == 0);
    for (size_t i = 0; i < params::CONFIG_deadzones.size(); i += 2) {
      laser->ClearDataInIndexRange(params::CONFIG_deadzones[i],
                                   params::CONFIG_deadzones[i + 1]);
    }
    if (kDebug) {
      modified_laser_pub_.publish(laser->ros_laser_scan_);
    }
  }

  void DrawPath(const cs::path_finding::Path2f& p) {
    robot_path_pub_.publish(
        visualization::DrawPath(p, params::CONFIG_map_tf_frame, "path"));
  }

  void DrawGoal(const util::Pose& goal) {
    visualization_msgs::MarkerArray goal_marker;
    visualization::DrawPose(goal,
                            params::CONFIG_map_tf_frame,
                            "goal_pose",
                            0,
                            1,
                            0,
                            1,
                            &goal_marker);
    goal_pub_.publish(goal_marker);
  }

  void DrawRobot(const util::vector_map::VectorMap& full_map,
                 util::Twist command) {
    robot_size_pub_.publish(
        visualization::MakeCylinder({0, 0},
                                    params::CONFIG_kRobotRadius,
                                    0.1,
                                    params::CONFIG_base_link_tf_frame,
                                    "robot_size",
                                    0,
                                    1,
                                    0,
                                    1,
                                    0.05));
    robot_size_pub_.publish(visualization::MakeCylinder(
        {params::CONFIG_kRobotRadius + params::CONFIG_kSafetyMargin, 0},
        0.05,
        0.1,
        params::CONFIG_base_link_tf_frame,
        "forward_bump",
        1,
        0,
        0,
        1,
        0.05));
    robot_size_pub_.publish(visualization::MakeCylinder(
        {0, 0},
        params::CONFIG_kRobotRadius + params::CONFIG_kSafetyMargin,
        0.1,
        params::CONFIG_base_link_tf_frame,
        "safety_size",
        0,
        0,
        1,
        0.1,
        0.05));

    const auto cd = util::physics::ComputeCommandDelta(
        state_estimator_->GetEstimatedPose(),
        state_estimator_->GetEstimatedVelocity(),
        command,
        state_estimator_->GetLaserTimeDelta());

    if (cd.type == util::physics::CommandDelta::Type::CURVE) {
      robot_size_pub_.publish(
          visualization::MakeCylinder(cd.curve.rotate_circle_center_wf,
                                      0.1,
                                      0.1,
                                      params::CONFIG_map_tf_frame,
                                      "rotatecenter",
                                      1,
                                      0,
                                      0,
                                      1));
    }

    const cs::motion_planning::TrajectoryRollout tr(
        state_estimator_->GetEstimatedPose(),
        state_estimator_->GetEstimatedVelocity(),
        command,
        state_estimator_->GetLaserTimeDelta());

    robot_size_pub_.publish(visualization::MakeCylinder(
        tr.final_pose.tra,
        params::CONFIG_kRobotRadius + params::CONFIG_kSafetyMargin,
        0.1,
        params::CONFIG_map_tf_frame,
        "final_safety",
        1,
        0,
        0,
        0.1,
        0.05));

    std::vector<util::Wall> colliding_walls;
    for (const auto& w : full_map.lines) {
      if (tr.IsColliding(
              w, params::CONFIG_kRobotRadius + params::CONFIG_kSafetyMargin)) {
        colliding_walls.push_back(w);
      }
    }
    robot_size_pub_.publish(visualization::DrawWalls(
        colliding_walls, params::CONFIG_map_tf_frame, "colliding_walls", 0.3));

    robot_size_pub_.publish(visualization::MakeCylinder(
        tr.final_pose.tra,
        params::CONFIG_kRobotRadius + params::CONFIG_kSafetyMargin,
        0.1,
        params::CONFIG_map_tf_frame,
        "final_safety",
        1,
        0,
        0,
        0.1,
        0.05));
  }

  util::Pose GetNextPose(const util::Pose& current_pose,
                         const cs::path_finding::Path2f& path) {
    if (path.waypoints.size() > 1) {
      return {path.waypoints[1], 0};
    }
    return current_pose;
  }

  util::Pose GetPoseFacingWaypoint(util::Pose p,
                                   const Eigen::Vector2f& waypoint) {
    const Eigen::Vector2f d = waypoint - p.tra;
    p.rot = math_util::AngleMod(std::atan2(d.y(), d.x()));
    return p;
  }

  void LaserCallback(const sensor_msgs::LaserScan& msg) {
    const auto laser_callback_start = GetMonotonicTime();
    util::LaserScan laser(msg);
    CleanLaserScan(&laser);
    state_estimator_->UpdateLaser(laser, msg.header.stamp);
    const auto est_pose = state_estimator_->GetEstimatedPose();
    const auto state_estimation_end = GetMonotonicTime();
    position_pub_.publish(est_pose.ToTwist());

    obstacle_detector_.UpdateObservation(est_pose, laser, &detected_walls_pub_);
    const auto obstacle_detection_end = GetMonotonicTime();

    if (motion_planner_.AtPose(current_goal_)) {
      ++current_goal_index_;
      current_goal_ = util::Pose(
          params::CONFIG_goal_poses[current_goal_index_ %
                                    params::CONFIG_goal_poses.size()]);
    }
    global_path_finder_.PlanPath(est_pose.tra, current_goal_.tra);
    const auto global_path = global_path_finder_.GetPath();
    DrawPath(global_path);
    const util::Pose global_waypoint = GetNextPose(est_pose, global_path);
    const auto global_path_plan_end = GetMonotonicTime();

    const auto local_path =
        local_path_finder_.FindPath(obstacle_detector_.GetDynamicFeatures(),
                                    est_pose.tra,
                                    global_waypoint.tra);
    DrawPath(local_path);
    const util::Pose local_waypoint = GetNextPose(
        GetPoseFacingWaypoint(est_pose, global_waypoint.tra), local_path);
    if (local_path.waypoints.empty()) {
      ROS_INFO("Local path planner failed");
    }
    const auto local_path_plan_end = GetMonotonicTime();

    DrawGoal(local_waypoint);
    util::Twist command = motion_planner_.DriveToPose(
        obstacle_detector_.GetDynamicFeatures(), local_waypoint);
    command_pub_.publish(command.ToTwist());
    const auto drive_to_pose_end = GetMonotonicTime();

    state_estimator_->UpdateLastCommand(command);

    PublishTransforms();
    if (kDebug) {
      state_estimator_->Visualize(&particle_pub_);
      map_pub_.publish(visualization::DrawWalls(map_.lines, "map", "map_ns"));
      DrawRobot(map_, command);
    }

    const auto laser_callback_end = GetMonotonicTime();

    const auto total_time = laser_callback_end - laser_callback_start;
    const auto state_estimation_time =
        state_estimation_end - laser_callback_start;
    const auto obstacle_detecton_time =
        obstacle_detection_end - state_estimation_end;
    const auto global_path_plan_time =
        global_path_plan_end - obstacle_detection_end;
    const auto local_path_plan_time =
        local_path_plan_end - global_path_plan_end;
    const auto drive_to_pose_time = drive_to_pose_end - local_path_plan_end;
    const auto draw_and_transfornms = laser_callback_end - drive_to_pose_end;
    std::cout << "total_time:             " << total_time << std::endl;
    std::cout << "state_estimation_time:  "
              << (state_estimation_time / total_time * 100) << "%" << std::endl;
    std::cout << "obstacle_detecton_time: "
              << (obstacle_detecton_time / total_time * 100) << "%"
              << std::endl;
    std::cout << "global_path_plan_time:  "
              << (global_path_plan_time / total_time * 100) << "%" << std::endl;
    std::cout << "local_path_plan_time:   "
              << (local_path_plan_time / total_time * 100) << "%" << std::endl;
    std::cout << "drive_to_pose_time:     "
              << (drive_to_pose_time / total_time * 100) << "%" << std::endl;
    std::cout << "draw_and_transfornms:   "
              << (draw_and_transfornms / total_time * 100) << "%" << std::endl;
  }

  void OdomCallback(const nav_msgs::Odometry& msg) {
    const util::Twist velocity(msg.twist.twist);
    state_estimator_->UpdateOdom(velocity, msg.header.stamp);
  }

  void PublishTransforms() {
    br_.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),
                                           ros::Time::now(),
                                           params::CONFIG_laser_tf_frame,
                                           params::CONFIG_base_link_tf_frame));

    const auto est_pose = state_estimator_->GetEstimatedPose();
    NP_FINITE_VEC(est_pose.tra);
    NP_FINITE(est_pose.rot);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(est_pose.tra.x(), est_pose.tra.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, est_pose.rot);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform.inverse(),
                                           ros::Time::now(),
                                           params::CONFIG_base_link_tf_frame,
                                           params::CONFIG_map_tf_frame));
  }
};

int main(int argc, char** argv) {
  std::string config_file = "src/control_stack/config/nav_config.lua";
  if (argc == 2) {
    config_file = argv[1];
  }
  ROS_INFO("Using config file: %s", config_file.c_str());
  config_reader::ConfigReader reader({config_file});
  ros::init(argc, argv, "nav_node");
  ros::NodeHandle n;
  CallbackWrapper cw(params::CONFIG_map, &n);
  ros::spin();
  return 0;
}
