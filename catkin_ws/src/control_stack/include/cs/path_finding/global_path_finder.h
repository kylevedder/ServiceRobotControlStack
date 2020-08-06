#pragma once
// Copyright 2020 kvedder@seas.upenn.edu
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

#include <eigen3/Eigen/Core>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "cs/path_finding/path_finder.h"
#include "cs/util/thread_safe/thread_safe_actor.h"

namespace cs {
namespace path_finding {

template <typename PF>
class GlobalPathFinder {
 private:
  PF path_finder_;
  util::threadsafe::ThreadSafeActor<std::pair<Eigen::Vector2f, Eigen::Vector2f>>
      start_goal_;
  util::threadsafe::ThreadSafeActor<cs::path_finding::Path2f> path_;
  cs::path_finding::Path2f last_path_;
  std::atomic_bool is_running_;
  std::thread planning_thread_;

  static constexpr int kSleepTime = 20;  // ms

  void DoPlanning() {
    while (is_running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTime));
      std::pair<Eigen::Vector2f, Eigen::Vector2f> start_goal = {
          Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()};
      if (!start_goal_.ReadOrDefault(&start_goal)) {
        continue;
      }
      auto path =
          path_finder_.FindPath({}, start_goal.first, start_goal.second);
      if (path.waypoints.empty()) {
        ROS_INFO("Global path planner from (%f, %f) to (%f, %f) failed",
                 start_goal.first.x(),
                 start_goal.first.y(),
                 start_goal.second.x(),
                 start_goal.second.y());
      }
      path_.Write(path);
    }
  }

 public:
  GlobalPathFinder(const util::vector_map::VectorMap& map,
                   const float& robot_radius,
                   const float& safety_margin,
                   const float& inflation)
      : path_finder_(map, robot_radius, safety_margin, inflation),
        start_goal_({Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero()}),
        path_(cs::path_finding::Path2f()),
        last_path_(),
        is_running_(true),
        planning_thread_(&GlobalPathFinder::DoPlanning, this) {}

  GlobalPathFinder(const GlobalPathFinder& other) = delete;
  GlobalPathFinder(GlobalPathFinder&& other) = delete;

  ~GlobalPathFinder() {
    is_running_ = false;
    planning_thread_.join();
  }

  void PlanPath(const Eigen::Vector2f& start, const Eigen::Vector2f& goal) {
    start_goal_.Write({start, goal});
  }

  Path2f GetPath() {
    path_.ReadOrDefault(&last_path_);
    return last_path_;
  }
};

}  // namespace path_finding
}  // namespace cs
