#pragma once
// Copyright 2018 - 2020 kvedder@umass.edu, kvedder@seas.upenn.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <vector>

namespace util {
namespace threadsafe {

template <class DataType>
class ThreadSafeActor {
 private:
  DataType data_;
  std::mutex access_mutex_;
  std::condition_variable read_cv_;
  std::atomic_bool is_updated_;
  std::atomic_bool is_running_;

  const float kSmallTime = 20;  // ms

 public:
  ThreadSafeActor() = delete;
  explicit ThreadSafeActor(const DataType& init_data)
      : data_(init_data), is_updated_(true), is_running_(true) {}

  ThreadSafeActor(const ThreadSafeActor<DataType>& other) = delete;
  ThreadSafeActor(ThreadSafeActor<DataType>&& other) = delete;

  ~ThreadSafeActor() {}

  void Shutdown() {
    is_running_ = false;
    is_updated_ = true;
    read_cv_.notify_all();
  }

  bool ReadOrDefault(DataType* output) {
    if (!is_updated_ || !is_running_) {
      return false;
    }
    // Transfer from the output queue to the local queue.
    std::unique_lock<std::mutex> guard(access_mutex_);

    // Copies data to output.
    *output = data_;

    // Reset the read flag.
    is_updated_ = false;

    read_cv_.notify_all();
    return true;  // Lock loses scope here.
  }

  void Write(const DataType& new_data) {
    if (!is_running_) {
      return;
    }
    // Transfer from the output actor to the local actor.
    std::unique_lock<std::mutex> guard(access_mutex_);
    data_ = new_data;
    is_updated_ = true;
    // Lock loses scope here.
  }
};
}  // namespace threadsafe
}  // namespace util
