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
#include "cs/path_finding/rrt.h"
#include "cs/state_estimation/pf_state_estimator.h"
#include "cs/state_estimation/sim_state_estimator.h"
#include "cs/state_estimation/state_estimator.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

static constexpr bool kDebug = true;

namespace params {
CONFIG_STRING(kMap, "pf.kMap");
CONFIG_FLOAT(kInitX, "pf.kInitX");
CONFIG_FLOAT(kInitY, "pf.kInitY");
CONFIG_FLOAT(kInitTheta, "pf.kInitTheta");
CONFIG_FLOAT(kGoalX, "pf.kGoalX");
CONFIG_FLOAT(kGoalY, "pf.kGoalY");
CONFIG_FLOAT(kRobotRadius, "pf.kRobotRadius");
CONFIG_FLOAT(kSafetyMargin, "pf.kSafetyMargin");
CONFIG_FLOAT(kCollisionRollout, "pf.kCollisionRollout");

CONFIG_STRING(map_tf_frame, "frames.map_tf_frame");
CONFIG_STRING(base_link_tf_frame, "frames.base_tf_frame");
CONFIG_STRING(laser_tf_frame, "frames.laser_tf_frame");

CONFIG_INTLIST(deadzones, "laser.deadzones");
CONFIG_BOOL(use_sim_ground_truth, "state_estimation.use_sim_ground_truth");
}  // namespace params

struct CallbackWrapper {
  util::Map map_;
  std::unique_ptr<cs::state_estimation::StateEstimator> state_estimator_;
  cs::obstacle_avoidance::ObstacleDetector obstacle_detector_;
  cs::motion_planning::PIDController motion_planner_;
  cs::path_finding::RRT path_finder_;
  tf::TransformBroadcaster br_;
  Eigen::Vector2f current_goal_;

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
  ros::Publisher robot_path_pub_;
  ros::Publisher base_link_robot_path_pub_;
  ros::Publisher rrt_tree_pub_;

  CallbackWrapper() = delete;

  cs::state_estimation::StateEstimator* MakeStateEstimator(ros::NodeHandle* n) {
    if (params::CONFIG_use_sim_ground_truth) {
      ROS_INFO("Using sim ground truth for state estimation");
      return new cs::state_estimation::SimStateEstimator(n);
    }
    ROS_INFO("Using PF for state estimation");
    return new cs::state_estimation::PFStateEstimator(
        map_,
        {params::CONFIG_kInitX,
         params::CONFIG_kInitY,
         params::CONFIG_kInitTheta});
  }

  CallbackWrapper(const std::string& map_file, ros::NodeHandle* n)
      : map_(map_file),
        state_estimator_(MakeStateEstimator(n)),
        obstacle_detector_(map_),
        motion_planner_(map_, *state_estimator_),
        path_finder_(
            map_, params::CONFIG_kRobotRadius, params::CONFIG_kSafetyMargin),
        current_goal_(params::CONFIG_kGoalX, params::CONFIG_kGoalY) {
    position_pub_ =
        n->advertise<geometry_msgs::Twist>(constants::kPositionTopic, 10);
    command_pub_ = n->advertise<geometry_msgs::Twist>(
        constants::kCommandVelocityTopic, 10);
    laser_sub_ = n->subscribe(
        constants::kLaserTopic, 10, &CallbackWrapper::LaserCallback, this);
    odom_sub_ = n->subscribe(
        constants::kOdomTopic, 10, &CallbackWrapper::OdomCallback, this);
    teleop_sub_ = n->subscribe(
        constants::kGoalTopic, 10, &CallbackWrapper::GoalCallback, this);

    if (kDebug) {
      modified_laser_pub_ =
          n->advertise<sensor_msgs::LaserScan>("scan_modified", 10);
      particle_pub_ =
          n->advertise<visualization_msgs::MarkerArray>("particles", 10);
      map_pub_ = n->advertise<visualization_msgs::Marker>("robot_map", 10);
      detected_walls_pub_ =
          n->advertise<visualization_msgs::Marker>("detected_obstacles", 10);
      robot_size_pub_ =
          n->advertise<visualization_msgs::Marker>("robot_size", 10);
      robot_path_pub_ =
          n->advertise<visualization_msgs::Marker>("robot_path", 10);
      base_link_robot_path_pub_ =
          n->advertise<visualization_msgs::Marker>("base_link_robot_path", 10);
      rrt_tree_pub_ = n->advertise<visualization_msgs::Marker>("rrt_tree", 10);
    }
  }

  void GoalCallback(const geometry_msgs::Pose2D& msg) {
    current_goal_ = {msg.x, msg.y};
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

  void LaserCallback(const sensor_msgs::LaserScan& msg) {
    util::LaserScan laser(msg);
    state_estimator_->UpdateLaser(laser, msg.header.stamp);
    const auto est_pose = state_estimator_->GetEstimatedPose();
    position_pub_.publish(est_pose.ToTwist());
    obstacle_detector_.UpdateObservation(est_pose, laser, &detected_walls_pub_);
    //    const auto path =
    //    path_finder_.FindPath(obstacle_detector_.GetDynamicMap(),
    //                                            est_pose.tra,
    //                                            current_goal_,
    //                                            &rrt_tree_pub_);
    //    const auto base_link_path =
    //    path.TransformPath((-est_pose).ToAffine());
    //    robot_path_pub_.publish(visualization::DrawPath(path, "map", "path"));
    //    base_link_robot_path_pub_.publish(
    //        visualization::DrawPath(base_link_path, "base_link", "path"));
    //    ROS_INFO("Dynamic map has %zu points",
    //             obstacle_detector_.GetDynamicMap().walls.size());
    const auto waypoint = current_goal_;
    //        (path.IsValid() ? path.waypoints[1]
    //                        : state_estimator_.GetEstimatedPose().tra);
    const auto desired_command = motion_planner_.DriveToPoint(
        obstacle_detector_.GetDynamicMap(), waypoint);

    command_pub_.publish(desired_command.ToTwist());
    state_estimator_->UpdateLastCommand(desired_command);
    PublishTransforms();
    if (kDebug) {
      state_estimator_->Visualize(&particle_pub_);
      map_pub_.publish(map_.ToMarker());
      robot_size_pub_.publish(
          visualization::MakeCylinder({0, 0},
                                      params::CONFIG_kRobotRadius,
                                      3.0,
                                      "/base_link",
                                      "robot_size",
                                      0,
                                      1,
                                      0,
                                      1));
      map_pub_.publish(map_.ToMarker());
    }
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
    NP_FINITE_2F(est_pose.tra);
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
  CallbackWrapper cw(params::CONFIG_kMap, &n);
  ros::spin();
  return 0;
}
