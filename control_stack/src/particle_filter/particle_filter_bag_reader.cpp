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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>

#include <string>
#include <utility>
#include <vector>

#include "config_reader/config_reader.h"
#include "cs/particle_filter/particle_filter.h"
#include "cs/util/constants.h"
#include "cs/util/pose.h"
#include "cs/util/util.h"

struct ParticleFilterWrapper {
  std::string error_file;
  localization::ParticleFilter particle_filter;
  util::Pose ground_truth;

  ParticleFilterWrapper() = delete;
  ParticleFilterWrapper(const std::string& error_file, const util::Map& map)
      : error_file(error_file), particle_filter(map) {
    std::ofstream out(error_file, std::fstream::out);
    out << "max_error_x,max_error_y,max_error_norm,max_error_theta,cent_error_"
           "x,cent_error_y,cent_error_norm,cent_error_theta\n";
    out.close();
  }

  void StartCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ground_truth = util::Pose(*msg);
    if (particle_filter.IsInitialized()) {
      return;
    }
    particle_filter.InitalizePose(ground_truth);
  }

  void WriteError(const util::Pose& max_estimate_error,
                  const util::Pose& weighted_centroid_error) const {
    std::ofstream out(error_file, std::fstream::out | std::fstream::app);
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

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    util::LaserScan laser(*msg);
    particle_filter.UpdateObservation(laser, nullptr);
    const util::Pose max_estimate = particle_filter.MaxWeight();
    const util::Pose weighted_centroid = particle_filter.WeightedCentroid();
    const util::Pose max_estimate_error = (max_estimate - ground_truth);
    const util::Pose weighted_centroid_error =
        (weighted_centroid - ground_truth);
    WriteError(max_estimate_error, weighted_centroid_error);
  }

  void OdomCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    util::Pose odom(*msg);
    particle_filter.UpdateOdom(odom.tra.x(), odom.rot);
  }
};

std::pair<std::string, std::vector<std::string>> GetParams(int argc,
                                                           char** argv) {
  NP_CHECK(argc > 2);
  std::vector<std::string> files;
  for (int i = 2; i < argc; ++i) {
    files.push_back({argv[i]});
  }
  return {argv[1], files};
}

int main(int argc, char** argv) {
  const auto config_files = GetParams(argc, argv);

  util::PrintCurrentWorkingDirectory();
  config_reader::ConfigReader reader(config_files.second);
  ParticleFilterWrapper wrapper(
      config_files.first,
      util::Map("/home/k/code/catkin_ws/src/particle_filter/maps/"
                "loop_small_bumps.map"));

  const std::string bag_name = "/home/k/code/catkin_ws/rosbags/loop.bag";
  rosbag::Bag bag(bag_name, rosbag::bagmode::Read);

  rosbag::View view(bag);
  ROS_INFO("Opened bag; found %d messages", view.size());
  for (rosbag::MessageInstance const& m : view) {
    if (m.getTopic() == "/true_pose") {
      geometry_msgs::Twist::ConstPtr msg =
          m.instantiate<geometry_msgs::Twist>();
      NP_CHECK(msg != nullptr);
      wrapper.StartCallback(msg);
    } else if (m.getTopic() == "/odom") {
      geometry_msgs::Twist::ConstPtr msg =
          m.instantiate<geometry_msgs::Twist>();
      NP_CHECK(msg != nullptr);
      wrapper.OdomCallback(msg);
    } else if (m.getTopic() == "/laser") {
      sensor_msgs::LaserScan::ConstPtr msg =
          m.instantiate<sensor_msgs::LaserScan>();
      NP_CHECK(msg != nullptr);
      wrapper.LaserCallback(msg);
    }
  }

  ROS_INFO("All messages read");

  return 0;
}
