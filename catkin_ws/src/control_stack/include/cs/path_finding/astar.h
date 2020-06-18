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

#include <math.h>
#include <algorithm>
#include <boost/functional/hash.hpp>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "cs/path_finding/path_finder.h"
#include "cs/util/constants.h"
#include "cs/util/visualization.h"
#include "libMultiRobotPlanning/bounded_a_star.hpp"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

namespace cs {
namespace path_finding {

namespace astar {
struct State {
  State() = delete;
  explicit State(Eigen::Vector2i pos) : pos(pos) {}
  State(int x, int y) : pos(x, y) {}

  State(const State&) = default;
  State(State&&) = default;
  State& operator=(const State&) = default;
  State& operator=(State&&) = default;

  bool operator==(const State& other) const { return pos == other.pos; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.pos.x() << "," << s.pos.y() << ")";
  }

  Eigen::Vector2i pos;

  inline int& x() { return pos.x(); }
  inline int& y() { return pos.y(); }
  inline const int& x() const { return pos.x(); }
  inline const int& y() const { return pos.y(); }
};

enum class Action {
  Up,
  Down,
  Left,
  Right,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:
      os << "Up";
      break;
    case Action::Down:
      os << "Down";
      break;
    case Action::Left:
      os << "Left";
      break;
    case Action::Right:
      os << "Right";
      break;
  }
  return os;
}

using Neigh = libMultiRobotPlanning::Neighbor<State, Action, int>;

template <int CellsPerMeter>
class Environment {
 public:
  Environment(State goal,
              const util::Map& map,
              const util::DynamicFeatures& df,
              const float min_distance_from_wall)
      : goal_(std::move(goal)),
        map_(map),
        df_(df),
        min_distance_from_wall_(min_distance_from_wall),
        transition_valid_timer_("transition_valid"),
        neighbor_generation_timer_("neighbor_generation") {}

  int admissibleHeuristic(const State& s) const {
    return Eigen::Vector2i(goal_.pos - s.pos).lpNorm<1>();
  }

  bool isSolution(const State& s) const { return s == goal_; }

  void getNeighbors(const State& s, std::vector<Neigh>& neighbors) {
    neighbors.clear();

    CumulativeFunctionTimer::Invocation invoke(&neighbor_generation_timer_);

    State up(s.x(), s.y() + 1);
    if (TransitionValid(s, up)) {
      neighbors.emplace_back(Neigh(up, Action::Up, 1));
    }
    State down(s.x(), s.y() - 1);
    if (TransitionValid(s, down)) {
      neighbors.emplace_back(Neigh(down, Action::Down, 1));
    }
    State left(s.x() - 1, s.y());
    if (TransitionValid(s, left)) {
      neighbors.emplace_back(Neigh(left, Action::Left, 1));
    }
    State right(s.x() + 1, s.y());
    if (TransitionValid(s, right)) {
      neighbors.emplace_back(Neigh(right, Action::Right, 1));
    }
  }

  void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

  void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

  static State WorldPositionToState(const Eigen::Vector2f& world_position) {
    const int cells_x =
        std::rintf(static_cast<float>(CellsPerMeter) * world_position.x());
    const int cells_y =
        std::rintf(static_cast<float>(CellsPerMeter) * world_position.y());
    return {cells_x, cells_y};
  }

  static Eigen::Vector2f StateToWorldPosition(const State& s) {
    return s.pos.cast<float>() * (1.0f / static_cast<float>(CellsPerMeter));
  }

 private:
  bool TransitionValid(const State& s1, const State& s2) {
    CumulativeFunctionTimer::Invocation invoke(&transition_valid_timer_);
    const auto p1 = StateToWorldPosition(s1);
    const auto p2 = StateToWorldPosition(s2);
    for (const auto& p : df_.features) {
      const float d = geometry::MinDistanceLineLine(p1, p2, p, p);
      if (d <= min_distance_from_wall_) {
        return false;
      }
    }
    for (const auto& w : map_.walls) {
      const float d = geometry::MinDistanceLineLine(p1, p2, w.p1, w.p2);
      if (d <= min_distance_from_wall_) {
        return false;
      }
    }
    return true;
  }

  State goal_;
  const util::Map& map_;
  const util::DynamicFeatures& df_;
  float min_distance_from_wall_;
  CumulativeFunctionTimer transition_valid_timer_;
  CumulativeFunctionTimer neighbor_generation_timer_;
};

}  // namespace astar

class AStar : public PathFinder {
 public:
  explicit AStar(const util::Map& map,
                 const float& robot_radius,
                 const float& safety_margin)
      : PathFinder(map, robot_radius, safety_margin) {}

  Path2f FindPath(const util::DynamicFeatures& dynamic_map,
                  const Eigen::Vector2f& start,
                  const Eigen::Vector2f& goal) override {
    if (!IsLineColliding(dynamic_map, start, goal)) {
      return {{start, goal}, (goal - start).norm()};
    }
    if (prev_path_.IsValid() && prev_path_.waypoints.back() == goal &&
        !IsPathColliding(dynamic_map, prev_path_)) {
      return SmoothPath(start, dynamic_map, prev_path_);
    }

    std::cout << "REPLAN" << std::endl;

    static constexpr int kCellsPerMeter = 2;
    static constexpr size_t kMaxExpansions = 10000;
    using Env = astar::Environment<kCellsPerMeter>;

    const auto start_state = Env::WorldPositionToState(start);
    const auto goal_state = Env::WorldPositionToState(goal);

    Env env(goal_state, map_, dynamic_map, robot_radius_ + safety_margin_);
    libMultiRobotPlanning::BoundedAStar<astar::State, astar::Action, int, Env>
        astar(env, kMaxExpansions);
    libMultiRobotPlanning::PlanResult<astar::State, astar::Action, int>
        solution;
    const bool success = astar.search(start_state, solution);
    if (!success) {
      std::cout << "Path planning failed!" << std::endl;
      return {};
    }

    Path2f path;
    for (const auto& sp : solution.states) {
      path.waypoints.push_back(Env::StateToWorldPosition(sp.first));
    }
    path.waypoints.push_back(goal);
    path.cost =
        static_cast<float>(solution.cost) / static_cast<float>(kCellsPerMeter);
    prev_path_ = path;
    return SmoothPath(start, dynamic_map, path);
  }
};

}  // namespace path_finding
}  // namespace cs

namespace std {
template <>
struct hash<cs::path_finding::astar::State> {
  size_t operator()(const cs::path_finding::astar::State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.pos.x());
    boost::hash_combine(seed, s.pos.y());
    return seed;
  }
};
}  // namespace std
