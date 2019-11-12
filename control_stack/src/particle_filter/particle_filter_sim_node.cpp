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
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>

#include "config_reader/config_reader.h"
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/obstacle_avoidance/trajectory_rollout.h"
#include "cs/particle_filter/particle_filter.h"
#include "cs/util/crash_handling.h"
#include "cs/util/math_util.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"

void DrawGroundTruth(const util::Pose& ground_truth,
                     ros::Publisher* ground_truth_pub) {
  visualization_msgs::MarkerArray arr;
  visualization::DrawPose(
      ground_truth, "map", "ground_truth", 0, 1, 0, 1, &arr);
  ground_truth_pub->publish(arr);
}

struct ParticleFilterWrapper {
  util::Map map;
  cs::localization::ParticleFilter particle_filter;
  cs::obstacle_avoidance::ObstacleDetector obstacle_detector;
  util::Pose ground_truth;
  ros::Publisher particle_pub;
  ros::Publisher ground_truth_pub;
  ros::Publisher sampled_laser_pub;
  ros::Publisher grid_belief_pub;
  ros::Publisher reference_pub;
  ros::Publisher dynamic_obs_pub;
  ros::Publisher dynamic_wall_pub;
  ros::Publisher trajectory_pub;

  static constexpr auto kErrorFile = "error.csv";

  ParticleFilterWrapper() = delete;
  ParticleFilterWrapper(const util::Map& map, ros::NodeHandle* n)
      : map(map), particle_filter(map), obstacle_detector(map) {
    particle_pub =
        n->advertise<visualization_msgs::MarkerArray>("particles", 10);
    ground_truth_pub =
        n->advertise<visualization_msgs::MarkerArray>("ground_truth", 10);
    sampled_laser_pub =
        n->advertise<sensor_msgs::LaserScan>("sampled_laser", 100);
    grid_belief_pub =
        n->advertise<visualization_msgs::MarkerArray>("grid_belief", 10);
    reference_pub =
        n->advertise<visualization_msgs::MarkerArray>("reference", 10);
    dynamic_obs_pub = n->advertise<visualization_msgs::MarkerArray>(
        "dynamic_observations", 10);
    dynamic_wall_pub =
        n->advertise<visualization_msgs::Marker>("dynamic_fitted_walls", 10);
    trajectory_pub =
        n->advertise<visualization_msgs::MarkerArray>("trajectories", 10);
    std::ofstream out(kErrorFile, std::fstream::out);
    out << "max_error_x,max_error_y,max_error_norm,max_error_theta,cent_error_"
           "x,cent_error_y,cent_error_norm,cent_error_theta\n";
    out.close();
  }

  void StartCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ground_truth = util::Pose(*msg);
    DrawGroundTruth(ground_truth, &ground_truth_pub);
    if (particle_filter.IsInitialized()) {
      return;
    }
    particle_filter.InitalizePose(ground_truth);
  }

  void WriteError(const util::Pose& max_estimate_error,
                  const util::Pose& weighted_centroid_error) const {
    std::ofstream out(kErrorFile, std::fstream::out | std::fstream::app);
    out << max_estimate_error.tra.x() << ", ";
    out << max_estimate_error.tra.y() << ", ";
    out << max_estimate_error.tra.norm() << ", ";
    out << max_estimate_error.rot << ", ";
    out << weighted_centroid_error.tra.x() << ", ";
    out << weighted_centroid_error.tra.y() << ", ";
    out << weighted_centroid_error.tra.norm() << ", ";
    out << weighted_centroid_error.rot << "\n";
    out.close();
  }

  void GridSearchBelief(const util::LaserScan& laser) {
    visualization_msgs::MarkerArray arr;
    float max_score = 0;
    static constexpr float kXMin = -0.75;
    static constexpr float kXMax = 0.75;
    static constexpr float kXDel = 0.1;
    static constexpr float kYMin = -0.75;
    static constexpr float kYMax = 0.75;
    static constexpr float kYDel = 0.1;
    static constexpr float kThetaMin = -kPi / 2;
    static constexpr float kThetaMax = kPi / 2 + kEpsilon;
    static constexpr float kThetaDel = kPi / 8;

    auto make_position =
        [this](const float x, const float y, const float theta) -> util::Pose {
      const Eigen::Vector2f offset_vector(x, y);
      return util::Pose(
          ground_truth.tra +
              Eigen::Rotation2Df(ground_truth.rot) * offset_vector,
          theta + ground_truth.rot);
    };

    for (float x = kXMin; x <= kXMax; x += kXDel) {
      for (float y = kYMin; y <= kYMax; y += kYDel) {
        for (float theta = kThetaMin; theta <= kThetaMax; theta += kThetaDel) {
          const util::Pose current_pose = make_position(x, y, theta);
          const float score =
              particle_filter.ScoreObservation(current_pose, laser);
          max_score = std::max(max_score, score);
        }
      }
    }

    for (float x = kXMin; x <= kXMax; x += kXDel) {
      for (float y = kYMin; y <= kYMax; y += kYDel) {
        for (float theta = kThetaMin; theta <= kThetaMax; theta += kThetaDel) {
          const Eigen::Vector2f offset_vector(x, y);
          const util::Pose current_pose = make_position(x, y, theta);
          const float score =
              particle_filter.ScoreObservation(current_pose, laser);
          const float red = score / max_score;
          visualization::DrawPose(
              current_pose, "map", "grid_search", red, 0, 0, 1, &arr, theta);
        }
      }
    }

    ROS_INFO("Published %zu grid elements", arr.markers.size());
    grid_belief_pub.publish(arr);

    visualization_msgs::MarkerArray ref_arr;
    const Eigen::Vector2f offset_tra(1, 0.5);
    const float rotation = 0;
    const util::Pose base_link_reference(offset_tra, rotation);
    visualization::DrawPose(
        base_link_reference, "base_link", "reference", 0, 0, 0, 1, &ref_arr);

    const util::Pose global_reference(
        ground_truth.tra + Eigen::Rotation2Df(ground_truth.rot) * offset_tra,
        rotation + ground_truth.rot);
    visualization::DrawPose(
        global_reference, "map", "reference", 1, 1, 1, 1, &ref_arr, 0.1);
    ref_arr.markers.push_back(visualization::LaserToLineList(
        laser, ground_truth, map, "map", "obs", 1, 0, 0, 1));
    reference_pub.publish(ref_arr);
  }

  void TestTrajectoryCollsions() {
    {
      cs::obstacle_avoidance::TrajectoryRollout tr(
          {{0, 0}, 0}, {{1, 0}, 0}, {{1, 0}, 0}, 2);
      util::Wall w1({3, -2}, {3, 2});
      util::Wall w2({1, -2}, {1, 2});
      util::Wall w3({2, -2}, {2, 2});
      util::Wall w4({2.2, -2}, {2.2, 2});
      NP_CHECK(!tr.IsColliding(w1, 0.1f));
      NP_CHECK(tr.IsColliding(w2, 0.1f));
      NP_CHECK(tr.IsColliding(w3, 0.1f));
      NP_CHECK(!tr.IsColliding(w4, 0.1f));
    }

    {
      const util::Pose start({0, 0}, 0);
      const util::Twist velocity({1, 0}, -kPi / 8);
      cs::obstacle_avoidance::TrajectoryRollout tr(
          start, velocity, velocity, 2);

      std::cout << "Achieved velocity pose: " << tr.achieved_vel_pose.tra.x()
                << ", " << tr.achieved_vel_pose.tra.y() << std::endl;

      std::cout << "Final pose:             " << tr.final_pose.tra.x() << ", "
                << tr.final_pose.tra.y() << std::endl;

      std::cout << "circle center:          " << tr.rotate_circle_center.x()
                << ", " << tr.rotate_circle_center.y() << std::endl;

      util::Wall w1({1, 1}, {1, -1});
      util::Wall w2({2, 1}, {2, -1});
      NP_CHECK(tr.IsColliding(w1, 0.1f));
      NP_CHECK(!tr.IsColliding(w2, 0.1f));
    }

    {
      std::cout << std::endl << std::endl << std::endl;
      const util::Pose start({-3, 0}, kPi / 2);
      const util::Twist velocity({1, 0}, -0.48);
      cs::obstacle_avoidance::TrajectoryRollout tr(
          start, velocity, velocity, 2);

      std::cout << "Achieved velocity pose: " << tr.achieved_vel_pose.tra.x()
                << ", " << tr.achieved_vel_pose.tra.y() << std::endl;

      std::cout << "Final pose:             " << tr.final_pose.tra.x() << ", "
                << tr.final_pose.tra.y() << std::endl;

      std::cout << "circle center:          " << tr.rotate_circle_center.x()
                << ", " << tr.rotate_circle_center.y() << std::endl;

      util::Wall w1({-2, 2}, {-2, -2});
      NP_CHECK(!tr.IsColliding(w1, 0.1f));
    }
    // NP_CHECK_VAL(false, "All unit tests completed!");
  }

  void TestTrajectories(const util::Pose& pose) {
    // TestTrajectoryCollsions();
    visualization_msgs::MarkerArray arr;
    static constexpr int kNumElems = 100;
    static constexpr float kRobotRadius = 0.1f;
    static constexpr float kRolloutTime = 2;

    const util::Twist velocity({1, 0}, 0);
    // static const util::Pose kInitialVelocity(0.9, 0, 0);

    for (int i = 0; i < kNumElems; ++i) {
      const float rot = -static_cast<float>(i - kNumElems / 2) /
                        (static_cast<float>(kNumElems) / 4.0f);
      //      const float rot = -0.48;
      const util::Twist commanded_velocity(1, 0, rot);
      cs::obstacle_avoidance::TrajectoryRollout tr(
          pose, velocity, commanded_velocity, kRolloutTime);
      bool is_colliding = false;
      for (const auto& wall : obstacle_detector.GetDynamicWalls()) {
        if (tr.IsColliding(wall, kRobotRadius)) {
          is_colliding = true;
          break;
        }
      }
      if (!is_colliding) {
        for (const auto& wall : map.walls) {
          if (tr.IsColliding(wall, kRobotRadius)) {
            is_colliding = true;
            break;
          }
        }
      }
      //      if (is_colliding) {
      //        std::cout << "Colliding angle: " << rot << std::endl;
      //      }
      visualization::DrawTrajectoryRollout(
          tr, "map", "test_tr", &arr, is_colliding);
    }
    trajectory_pub.publish(arr);
  }

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    util::LaserScan laser(*msg);
    particle_filter.UpdateObservation(laser, &sampled_laser_pub);
    particle_filter.DrawParticles(&particle_pub);
    const util::Pose max_estimate = particle_filter.MaxWeight();
    const util::Pose weighted_centroid = particle_filter.WeightedCentroid();
    const util::Pose max_estimate_error = (max_estimate - ground_truth);
    const util::Pose weighted_centroid_error =
        (weighted_centroid - ground_truth);
    WriteError(max_estimate_error, weighted_centroid_error);

    GridSearchBelief(laser);
    obstacle_detector.UpdateObservation(
        weighted_centroid, laser, &dynamic_obs_pub);
    obstacle_detector.DrawDynamic(&dynamic_wall_pub);
    TestTrajectories(weighted_centroid);
  }

  void OdomCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    util::Pose odom(*msg);
    particle_filter.UpdateOdom(odom.tra.x(), odom.rot);
    particle_filter.DrawParticles(&particle_pub);
  }
};

int main(int argc, char** argv) {
  CONFIG_STRING(kMap, "pf.kMap");

  util::PrintCurrentWorkingDirectory();
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/pf_sim_config.lua"});
  ros::init(argc, argv, "particle_filter", ros::init_options::NoSigintHandler);

  if (signal(SIGINT, util::crash::FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGINT" << std::endl;
    exit(-1);
  }
  if (signal(SIGSEGV, util::crash::FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGSEGV" << std::endl;
    exit(-1);
  }
  if (signal(SIGABRT, util::crash::FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGABRT" << std::endl;
    exit(-1);
  }

  ros::NodeHandle n;

  ParticleFilterWrapper wrapper(util::Map(kMap), &n);

  ros::Subscriber initial_pose_sub = n.subscribe(
      "true_pose", 1000, &ParticleFilterWrapper::StartCallback, &wrapper);
  ros::Subscriber laser_sub = n.subscribe(
      "laser", 1000, &ParticleFilterWrapper::LaserCallback, &wrapper);
  ros::Subscriber odom_sub =
      n.subscribe("odom", 1000, &ParticleFilterWrapper::OdomCallback, &wrapper);

  ros::spin();

  return 0;
}
