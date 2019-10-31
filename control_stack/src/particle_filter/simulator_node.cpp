#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "cs/util/constants.h"
#include "cs/util/geometry.h"
#include "cs/util/map.h"
#include "cs/util/math_util.h"
#include "cs/util/pose.h"
#include "cs/util/util.h"
#include "cs/util/visualization.h"

#include "config_reader/config_reader.h"

#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <random>
#include <string>

namespace sim {
CONFIG_FLOAT(kLaserStdDev, "sim.kLaserStdDev");
CONFIG_FLOAT(kArcExecStdDev, "sim.kArcExecStdDev");
CONFIG_FLOAT(kArcReadStdDev, "sim.kArcReadStdDev");
CONFIG_FLOAT(kRotateExecStdDev, "sim.kRotateExecStdDev");
CONFIG_FLOAT(kRotateReadStdDev, "sim.kRotateReadStdDev");

}  // namespace sim

// std::random_device rd;
std::mt19937 gen(0);

std_msgs::Header MakeHeader(const std::string& frame_id) {
  static uint32_t seq = 0;
  std_msgs::Header header;
  header.seq = (++seq);
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  return header;
}

sensor_msgs::LaserScan MakeScan(const util::Pose& robot_pose,
                                const util::Map& map,
                                const float noise_stddev) {
  std::normal_distribution<> noise_dist(0.0f, noise_stddev);

  sensor_msgs::LaserScan scan;
  scan.header = MakeHeader("base_link");
  scan.angle_min = kMinAngle;
  scan.angle_max = kMaxAngle;
  scan.angle_increment = kAngleDelta;
  scan.range_min = kMinReading;
  scan.range_max = kMaxReading;
  scan.scan_time = 0;
  scan.time_increment = 0;

  for (int ray_idx = 0; ray_idx < kNumReadings; ++ray_idx) {
    const float angle = math_util::AngleMod(
        kMinAngle + kAngleDelta * static_cast<float>(ray_idx) + robot_pose.rot);
    const util::Pose ray(robot_pose.tra, angle);
    const float dist =
        map.MinDistanceAlongRay(ray, kMinReading, kMaxReading - kEpsilon);
    scan.ranges.push_back(dist + noise_dist(gen));
  }

  return scan;
}

void PublishTransforms(const util::Pose& current_pose) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(
      tf::Vector3(current_pose.tra.x(), current_pose.tra.y(), 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, current_pose.rot);
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

util::Pose AddExecutionOdomNoise(util::Pose move) {
  std::normal_distribution<> along_arc_dist(
      0.0f, sim::kArcExecStdDev * move.tra.norm());
  std::normal_distribution<> rotation_dist(
      0.0f, sim::kRotateExecStdDev * move.rot + 0.005);
  move.tra.x() += along_arc_dist(gen);
  move.rot += rotation_dist(gen);
  return move;
}

util::Pose AddReadingOdomNoise(util::Pose move) {
  std::normal_distribution<> along_arc_dist(
      0.0f, sim::kArcReadStdDev * move.tra.norm());
  std::normal_distribution<> rotation_dist(0.0f,
                                           sim::kRotateReadStdDev * move.rot);
  move.tra.x() += along_arc_dist(gen);
  move.rot += rotation_dist(gen);
  return move;
}

util::Pose CommandSpinCircle(const util::Pose& current_pose) {
  return {{0.05f, 0}, -0.1f};
}

util::Pose DriveToPose(const util::Pose& current_pose,
                       const util::Pose& goal_pose) {
  static constexpr bool kDebug = false;
  const util::Pose delta = goal_pose - current_pose;
  static float kMinRotationalErrorToHalt = kPi / 4;
  static float kRotP = 0.5f;
  static float kMaxRot = 0.1f;
  static float kTraP = 0.5f;
  static float kMaxTra = 0.1f;
  static float kTraHaltThreshold = 0.25f;
  static float kTraRotThreshold = 0.1f;

  // At goal translationally and rotationally.
  if (delta.tra.squaredNorm() < Sq(kTraHaltThreshold) &&
      fabs(delta.rot) < kTraRotThreshold) {
    if (kDebug) {
      ROS_INFO("Drive complete!");
    }
    return {{0, 0}, 0};
  }

  // At goal translationally but not rotationally.
  if (delta.tra.squaredNorm() < Sq(kTraHaltThreshold)) {
    if (kDebug) {
      ROS_INFO("Rotate to final!");
    }
    const float rot_cmd =
        math_util::Sign(delta.rot) * std::min(kMaxRot, fabs(delta.rot) * kRotP);
    return {{0, 0}, rot_cmd};
  }

  const Eigen::Vector2f current_to_goal_norm = delta.tra.normalized();
  const float desired_angle = math_util::AngleMod(
      atan2(current_to_goal_norm.y(), current_to_goal_norm.x()));
  const float angle_to_goal = std::min(
      math_util::AngleMod(desired_angle - current_pose.rot),
      math_util::AngleMod(math_util::AngleMod(desired_angle + kPi) -
                          math_util::AngleMod(current_pose.rot + kPi)));
  if (kDebug) {
    ROS_INFO("Desired angle %f  Current Angle %f, Angle to goal: %f",
             desired_angle, current_pose.rot, angle_to_goal);
  }

  const float rot_cmd = math_util::Sign(angle_to_goal) *
                        std::min(kMaxRot, fabs(angle_to_goal) * kRotP);

  // If angle is too far away to correct while moving.
  if (fabs(angle_to_goal) > kMinRotationalErrorToHalt) {
    if (kDebug) {
      ROS_INFO("Correct angle to drive!");
    }
    return {{0, 0}, rot_cmd};
  }

  const Eigen::Vector2f tra_cmd = [&delta]() -> Eigen::Vector2f {
    const Eigen::Vector2f uncapped_tra_cmd =
        Eigen::Rotation2Df(-delta.rot) * (delta.tra * kTraP);
    if (uncapped_tra_cmd.squaredNorm() > Sq(kMaxTra)) {
      return {kMaxTra, 0};
    }
    return {fabs(uncapped_tra_cmd.x()), 0};
  }();
  if (kDebug) {
    ROS_INFO("Drive to goal!");
  }
  return {tra_cmd, rot_cmd};
}

util::Pose CommandPointsLoop(const util::Pose& current_pose,
                             const std::vector<util::Pose>& waypoints) {
  NP_CHECK(!waypoints.empty());
  static size_t current_waypoint_idx = 0;
  current_waypoint_idx = current_waypoint_idx % waypoints.size();
  const util::Pose& current_waypoint = waypoints.at(current_waypoint_idx);
  const util::Pose cmd = DriveToPose(current_pose, current_waypoint);
  if (cmd != util::Pose({0, 0}, 0)) {
    return cmd;
  }
  current_waypoint_idx = (current_waypoint_idx + 1) % waypoints.size();
  return DriveToPose(current_pose, waypoints.at(current_waypoint_idx));
}

int main(int argc, char** argv) {
  CONFIG_STRING(kMap, "sim.kMap");
  util::PrintCurrentWorkingDirectory();
  config_reader::ConfigReader reader(
      {"src/ServiceRobotControlStack/control_stack/config/pf_sim_config.lua",
       "src/ServiceRobotControlStack/control_stack/config/sim_config.lua"});
  ros::init(argc, argv, "simulator");

  ros::NodeHandle n;

  ros::Publisher initial_pose_pub =
      n.advertise<geometry_msgs::Twist>("true_pose", 1);
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("laser", 10);
  ros::Publisher odom_pub = n.advertise<geometry_msgs::Twist>("odom", 10);
  ros::Publisher map_pub = n.advertise<visualization_msgs::Marker>("map", 10);
  ros::Publisher initial_pose_vis_pub =
      n.advertise<visualization_msgs::MarkerArray>("true_pose_vis", 1);

  ros::Rate loop_rate(10);

  const util::Map map(kMap);
  const std::vector<util::Pose> waypoints = {
      {{3.5, -3.5}, kPi},
      {{-3.5, -3.5}, kPi / 2},
      {{-3.5, 3.5}, 0},
      {{3.5, 3.5}, -kPi / 2},
  };

  util::Pose current_pose = waypoints.front();

  while (ros::ok()) {
    const util::Pose desired_move = CommandPointsLoop(current_pose, waypoints);
    const util::Pose executed_move = AddExecutionOdomNoise(desired_move);
    const util::Pose reported_move = AddReadingOdomNoise(executed_move);
    current_pose = geometry::FollowTrajectory(
        current_pose, executed_move.tra.x(), executed_move.rot);

    PublishTransforms(current_pose);
    scan_pub.publish(MakeScan(current_pose, map, sim::kLaserStdDev));
    odom_pub.publish(reported_move.ToTwist());
    initial_pose_pub.publish(current_pose.ToTwist());
    map_pub.publish(map.ToMarker());
    visualization_msgs::MarkerArray arr;
    visualization::DrawPose(current_pose, "map", "true_pose_vis", 1, 1, 1, 1,
                            &arr);
    initial_pose_vis_pub.publish(arr);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}