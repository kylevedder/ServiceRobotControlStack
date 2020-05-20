#pragma once
// Copyright 2017 - 2019 kvedder@seas.upenn.edu
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

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include "cs/util/constants.h"

namespace array_util {

const bool kProduction = false;

template <size_t N, class T>
std::array<T, N> MakeArray(const T& v) {
  std::array<T, N> ret;
  ret.fill(v);
  return ret;
}

template <size_t N, class T>
T SumArray(const std::array<T, N>& v) {
  T acc = 0;
  for (const auto& e : v) {
    acc += e;
  }
  return acc;
}

template <size_t N, class T>
T SelectiveSumArray(const std::array<T, N>& v,
                    const std::array<bool, N>& b,
                    const T zero = 0) {
  T acc = zero;
  for (size_t i = 0; i < N; ++i) {
    const auto& e = v[i];
    if (b[i]) {
      acc += e;
    }
  }
  return acc;
}

template <size_t N, class T>
bool SelectiveEqual(const std::array<bool, N>& b,
                    const std::array<T, N>& v1,
                    const std::array<T, N>& v2) {
  for (size_t i = 0; i < N; ++i) {
    if (b[i] && v1[i] != v2[i]) {
      return false;
    }
  }
  return true;
}

template <size_t N, class T>
T MinElement(const std::array<T, N>& v) {
  T acc = std::numeric_limits<T>::max();
  for (const auto& e : v) {
    acc = std::min(acc, e);
  }
  return acc;
}

template <size_t N, class T>
T SelectiveMinElement(const std::array<T, N>& v, const std::array<bool, N>& b) {
  T acc = std::numeric_limits<T>::max();
  for (size_t i = 0; i < N; ++i) {
    if (!b[i]) {
      continue;
    }
    const auto& e = v[i];
    acc = std::min(acc, e);
  }
  return acc;
}

template <size_t N, class T>
T MaxElement(const std::array<T, N>& v) {
  T acc = std::numeric_limits<T>::min();
  for (const auto& e : v) {
    acc = std::max(acc, e);
  }
  return acc;
}

template <size_t N, class T>
T SelectiveMaxElement(const std::array<T, N>& v, const std::array<bool, N>& b) {
  T acc = std::numeric_limits<T>::min();
  for (size_t i = 0; i < N; ++i) {
    if (!b[i]) {
      continue;
    }
    const auto& e = v[i];
    acc = std::max(acc, e);
  }
  return acc;
}

template <size_t N, class T>
std::array<T, N> AddToEachElement(const std::array<T, N>& a1, const T& val) {
  std::array<T, N> ret;
  for (size_t i = 0; i < N; ++i) {
    ret[i] = a1[i] + val;
  }
  return ret;
}

template <size_t N, class T>
std::array<T, N> AddArrayElements(const std::array<T, N>& a1,
                                  const std::array<T, N>& a2) {
  std::array<T, N> ret;
  for (size_t i = 0; i < N; ++i) {
    ret[i] = a1[i] + a2[i];
  }
  return ret;
}

template <size_t N, class T>
std::array<T, N> SubtractArrayElements(const std::array<T, N>& a1,
                                       const std::array<T, N>& a2) {
  std::array<T, N> ret;
  for (size_t i = 0; i < N; ++i) {
    ret[i] = a1[i] - a2[i];
  }
  return ret;
}

template <size_t N, class T>
std::array<T, N> GetIndexedElements(const std::array<std::vector<T>, N>& a,
                                    const std::array<bool, N>& needs_replans,
                                    const std::array<std::size_t, N>& indices,
                                    const T& zero) {
  std::array<T, N> ret = MakeArray<N>(zero);
  for (size_t i = 0; i < N; ++i) {
    if (needs_replans[i]) {
      const auto& v = a[i];
      const size_t& index = indices[i];
      NP_CHECK(index < v.size());
      ret[i] = v[index];
    }
  }
  return ret;
}

template <size_t N, class Container>
size_t MaxDatastructureSize(const std::array<Container, N>& v) {
  size_t max = 0;
  for (const auto& e : v) {
    max = std::max(max, e.size());
  }
  return max;
}

}  // namespace array_util
