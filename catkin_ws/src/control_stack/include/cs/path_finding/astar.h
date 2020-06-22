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
  UpLeft,
  UpRight,
  DownLeft,
  DownRight,
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
    case Action::UpLeft:
      os << "UpLeft";
      break;
    case Action::DownLeft:
      os << "DownLeft";
      break;
    case Action::UpRight:
      os << "UpRight";
      break;
    case Action::DownRight:
      os << "DownRight";
      break;
  }
  return os;
}

using Neigh = libMultiRobotPlanning::Neighbor<State, Action, float>;

template <int CellsPerMeter, bool UseEightGrid>
class Environment {
 public:
  Environment(State goal,
              const util::vector_map::VectorMap& map,
              const util::DynamicFeatures& df,
              const float min_distance_from_wall)
      : goal_(std::move(goal)),
        map_(map),
        df_(df),
        min_distance_from_wall_(min_distance_from_wall) {}

  float admissibleHeuristic(const State& s) const {
    if (UseEightGrid) {
      static constexpr float kInflation = 1.05f;
      const Eigen::Vector2i delta = (goal_.pos - s.pos).cwiseAbs();
      //      return delta.lpNorm<2>();

      if (delta.x() > delta.y()) {
        return kInflation * (delta.y() * (kSqrtTwo - 1) + delta.x());
      }
      return kInflation * (delta.x() * (kSqrtTwo - 1) + delta.y());
    } else {
      return Eigen::Vector2i(goal_.pos - s.pos).lpNorm<1>();
    }
  }

  bool isSolution(const State& s) const { return s == goal_; }

  void getNeighbors(const State& s, std::vector<Neigh>& neighbors) {
    neighbors.clear();

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

    if (UseEightGrid) {
      State up_left(s.x() - 1, s.y() + 1);
      if (TransitionValid(s, up_left)) {
        neighbors.emplace_back(Neigh(up_left, Action::UpLeft, kSqrtTwo));
      }
      State up_right(s.x() + 1, s.y() + 1);
      if (TransitionValid(s, up_right)) {
        neighbors.emplace_back(Neigh(up_right, Action::UpRight, kSqrtTwo));
      }
      State down_left(s.x() - 1, s.y() - 1);
      if (TransitionValid(s, down_left)) {
        neighbors.emplace_back(Neigh(down_left, Action::DownLeft, kSqrtTwo));
      }
      State down_right(s.x() + 1, s.y() - 1);
      if (TransitionValid(s, down_right)) {
        neighbors.emplace_back(Neigh(down_right, Action::DownRight, kSqrtTwo));
      }
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
    const auto p1 = StateToWorldPosition(s1);
    const auto p2 = StateToWorldPosition(s2);
    for (const auto& p : df_.features) {
      if (geometry::Line2f(p, p).CloserThan(p1, p2, min_distance_from_wall_)) {
        return false;
      }
    }
    for (const auto& w : map_.lines) {
      if (w.CloserThan(p1, p2, min_distance_from_wall_)) {
        return false;
      }
    }
    return true;
  }

  State goal_;
  const util::vector_map::VectorMap& map_;
  const util::DynamicFeatures& df_;
  float min_distance_from_wall_;
};

}  // namespace astar

template <int CellsPerMeter, size_t MaxExpansions, bool UseEightGrid>
class AStar : public PathFinder {
 public:
  explicit AStar(const util::vector_map::VectorMap& map,
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

    using Env = astar::Environment<CellsPerMeter, UseEightGrid>;

    const auto start_state = Env::WorldPositionToState(start);
    const auto goal_state = Env::WorldPositionToState(goal);

    Env env(goal_state,
            map_,
            dynamic_map,
            robot_radius_ + safety_margin_ + kEpsilon);
    libMultiRobotPlanning::BoundedAStar<astar::State, astar::Action, float, Env>
        astar(env, MaxExpansions);
    libMultiRobotPlanning::PlanResult<astar::State, astar::Action, float>
        solution;
    if (!astar.search(start_state, solution)) {
      return {};
    }

    Path2f path;
    path.waypoints.reserve(solution.states.size() + 1);
    for (const auto& sp : solution.states) {
      path.waypoints.push_back(Env::StateToWorldPosition(sp.first));
    }
    path.waypoints.push_back(goal);
    path.cost =
        static_cast<float>(solution.cost) / static_cast<float>(CellsPerMeter);
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
