#pragma once
// Copyright 2017 - 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
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

#include <array>

#include "cs/util/constants.h"

namespace cs {
namespace datastructures {
template <typename Value, size_t Size>
class CircularBuffer {
 private:
  size_t current_size_;
  size_t head_index_;
  size_t tail_index_;
  std::array<Value, Size> manual_queue_;

  inline size_t WrapIndex(const size_t unwrapped_index) const {
    return unwrapped_index % Size;
  }

 public:
  CircularBuffer() : current_size_(0), head_index_(0), tail_index_(0) {}

  ~CircularBuffer() {}

  void clear() {
    current_size_ = 0;
    head_index_ = 0;
    tail_index_ = 0;
  }

  Value& operator[](size_t i) {
    const auto idx = WrapIndex(head_index_ + i);
    NP_CHECK(idx < Size);
    return manual_queue_[idx];
  }

  const Value& operator[](size_t i) const {
    const auto idx = WrapIndex(head_index_ + i);
    NP_CHECK(idx < Size);
    return manual_queue_[idx];
  }

  CircularBuffer<Value, Size>& operator=(
      const CircularBuffer<Value, Size>& other) = default;
  CircularBuffer<Value, Size>& operator=(CircularBuffer<Value, Size>&& other) =
      default;

  size_t size() const { return current_size_; }

  void push_back(const Value& v) {
    const size_t idx = (empty()) ? tail_index_ : WrapIndex(tail_index_ + 1);
    manual_queue_[idx] = v;
    tail_index_ = idx;
    if (current_size_ < Size) {
      ++current_size_;
    } else {
      head_index_ = WrapIndex(head_index_ + 1);
    }
  }

  bool empty() const { return (current_size_ == 0); }

  Value& front() { return (*this)[0]; }

  const Value& front() const { return (*this)[0]; }

  Value& back() { return (*this)[current_size_ - 1]; }

  const Value& back() const { return (*this)[current_size_ - 1]; }

  class CircularBufferIterator {
   private:
    CircularBuffer* buffer_;
    size_t index_;

   public:
    CircularBufferIterator() = delete;

    CircularBufferIterator(CircularBuffer* buffer, const size_t index)
        : buffer_(buffer), index_(index) {}

    bool operator==(const CircularBufferIterator& other) const {
      return (buffer_ == other.buffer_) && (index_ == other.index_);
    }

    bool operator!=(const CircularBufferIterator& other) const {
      return !(*this == other);
    }

    void operator++() { index_++; }

    const Value& operator*() const { return (*buffer_)[index_]; }

    Value& operator*() {
      Value& v = (*buffer_)[index_];
      return v;
    }
  };

  CircularBufferIterator begin() { return CircularBufferIterator(this, 0); }

  CircularBufferIterator begin() const {
    return CircularBufferIterator(this, 0);
  }

  CircularBufferIterator end() {
    return CircularBufferIterator(this, current_size_);
  }

  CircularBufferIterator end() const {
    return CircularBufferIterator(this, current_size_);
  }
};

}  // namespace datastructures
}  // namespace cs
