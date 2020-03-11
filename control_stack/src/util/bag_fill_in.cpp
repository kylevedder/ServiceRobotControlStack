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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <fstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "cs/util/point_cloud.h"

using PC16 = util::pc::PointCloud<util::pc::Point16>;
using PC20 = util::pc::PointCloud<util::pc::Point20>;

std::tuple<std::string, std::string> GetArgs(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: [base file] [object file]" << std::endl;
    exit(0);
  }
  return {argv[1], argv[2]};
}

PC16 AveragePC(rosbag::Bag* bag) {
  std::vector<PC16> pcs;
  for (const auto& m : rosbag::View(*bag)) {
    if (m.getTopic() == "/camera/depth/points") {
      auto s = m.instantiate<sensor_msgs::PointCloud2>();
      if (s == nullptr) {
        continue;
      }
      auto pc = PC16(*s);
      return pc;
      pcs.push_back(pc);
    }
  }

  CHECK(!pcs.empty());

  PC16 acc_pc = pcs.front();
  for (auto pcs_it = pcs.begin() + 1; pcs_it != pcs.end(); ++pcs_it) {
    std::pair<util::pc::Point16*, util::pc::Point16 const*> it_pair = {
        acc_pc.begin(), pcs_it->begin()};
    const std::pair<util::pc::Point16*, util::pc::Point16 const*> it_pair_end =
        {acc_pc.end(), pcs_it->end()};

    for (; it_pair != it_pair_end;
         it_pair = {++it_pair.first, ++it_pair.second}) {
      auto* acc_it = it_pair.first;
      const auto* curr_it = it_pair.second;
      acc_it->x += curr_it->x;
      acc_it->y += curr_it->y;
      acc_it->z += curr_it->z;
    }
  }

  const float divisor = static_cast<float>(pcs.size());
  for (auto& p : acc_pc) {
    p.x /= divisor;
    p.y /= divisor;
    p.z /= divisor;
  }

  return acc_pc;
}

PC16 DiffPCs(const PC16& base_pc, PC16 object_pc, const bool keep_same) {
  std::pair<util::pc::Point16 const*, util::pc::Point16*> it_pair = {
      base_pc.begin(), object_pc.begin()};
  const std::pair<util::pc::Point16 const*, util::pc::Point16*> it_pair_end = {
      base_pc.end(), object_pc.end()};
  for (; it_pair != it_pair_end;
       it_pair = {++it_pair.first, ++it_pair.second}) {
    const auto* base_pt = it_pair.first;
    auto* object_pt = it_pair.second;

    const auto dist_sq =
        (base_pt->GetMappedVector3f() - object_pt->GetMappedVector3f())
            .squaredNorm();

    static constexpr float kMaxDist = 0.05;  // Meters

    if (!base_pt->IsValid() || !object_pt->IsValid()) {
      object_pt->Invalidate();
      continue;
    }

    if (keep_same) {
      if (dist_sq > kMaxDist) {
        object_pt->Invalidate();
      }
    } else {
      if (dist_sq < kMaxDist) {
        object_pt->Invalidate();
      }
    }
  }

  return object_pc;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bag_fill_in");
  auto res = GetArgs(argc, argv);
  rosbag::Bag base_bag(std::get<0>(res));
  rosbag::Bag object_bag(std::get<1>(res));
  PC16 base_pc = AveragePC(&base_bag);
  PC16 object_pc = AveragePC(&object_bag);

  static const Eigen::Affine3f transform =
      (Eigen::Affine3f::Identity() * Eigen::Translation3f(0, 0, 0.78) *
       Eigen::AngleAxisf(-kPi / 2, Eigen::Vector3f::UnitZ())) *
      Eigen::AngleAxisf(-kPi / 2.05, Eigen::Vector3f::UnitX());

  base_pc.TransformFrame(transform, "robot_frame");
  object_pc.TransformFrame(transform, "robot_frame");

  const PC16 object_only_pc = DiffPCs(base_pc, object_pc, false);
  const PC16 no_object_pc = DiffPCs(base_pc, object_pc, true);

  ros::NodeHandle n;

  auto object_only_pub =
      n.advertise<sensor_msgs::PointCloud2>("/object_only", 10);
  auto no_object_pub = n.advertise<sensor_msgs::PointCloud2>("/no_object", 10);
  auto plane_pub = n.advertise<visualization_msgs::Marker>("/ground_plane", 10);

  while (ros::ok()) {
    object_only_pub.publish(object_only_pc.GetRosPC());
    no_object_pub.publish(no_object_pc.GetRosPC());
    ros::spinOnce();
  }
  //  CallbackWrapper cw;
  //  std::ofstream laser_file(std::get<1>(res));
  //  std::ofstream commands_file(std::get<2>(res));
  //  int iter = 0;
  //  for (const auto& m : rosbag::View(bag)) {
  //    if (m.getTopic() == "/scan") {
  //      auto s = m.instantiate<sensor_msgs::LaserScan>();
  //      CHECK(s != nullptr);
  //      auto commands = cw.LaserCallback(*s);
  //      WriteResults(&laser_file,
  //                   &commands_file,
  //                   &dynamic_map_file,
  //                   &trajectory_file,
  //                   std::get<0>(commands),
  //                   std::get<1>(commands),
  //                   std::get<2>(commands));
  //      ++iter;
  //    }
  //  }
  //  bag.close();
  //  laser_file.close();
  //  commands_file.close();

  return 0;
}
