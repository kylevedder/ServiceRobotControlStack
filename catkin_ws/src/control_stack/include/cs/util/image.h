#pragma once
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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <utility>

#include "cs/util/constants.h"

namespace util {
namespace img {

struct __attribute__((packed)) PixelRGB8 {
  std::uint8_t r;
  std::uint8_t g;
  std::uint8_t b;

  PixelRGB8() = delete;

  static std::string EncodingType() {
    return sensor_msgs::image_encodings::RGB8;
  }
};
static_assert(sizeof(PixelRGB8) == 3, "PixelRBG8 must be 3 bytes");

struct __attribute__((packed)) PixelMONO8 {
  std::uint8_t d;

  PixelMONO8() = delete;

  static std::string EncodingType() {
    return sensor_msgs::image_encodings::MONO8;
  }
};
static_assert(sizeof(PixelMONO8) == 1, "PixelMONO8 must be 1 byte");

struct __attribute__((packed)) PixelMONO16 {
  std::uint16_t d;

  PixelMONO16() = delete;

  static std::string EncodingType() {
    return sensor_msgs::image_encodings::MONO16;
  }
};
static_assert(sizeof(PixelMONO16) == 2, "PixelMONO16 must be 2 bytes");

template <typename Pixel>
class Image {
  sensor_msgs::Image ros_img_;

  void VerifyEncoding() const {
    NP_CHECK_EQ(ros_img_.encoding, Pixel::EncodingType());
    NP_CHECK_EQ(ros_img_.width * sizeof(Pixel), ros_img_.step);
  }

 public:
  Image() = default;
  explicit Image(const sensor_msgs::Image& ros_img) : ros_img_(ros_img) {
    VerifyEncoding();
  }

  void operator=(const sensor_msgs::Image& ros_img) {
    ros_img_ = ros_img;
    VerifyEncoding();
  }

  Pixel* At(const int x, const int y) {
    NP_CHECK(x < Width());
    NP_CHECK(y < Height());
    return (begin() + y * Width() + x);
  }

  const Pixel& At(const int x, const int y) const {
    NP_CHECK(x < Width());
    NP_CHECK(y < Height());
    return *(begin() + y * Width() + x);
  }

  int Width() const { return ros_img_.width; }

  int Height() const { return ros_img_.height; }

  std::pair<int, int> Shape() const { return {Width(), Height()}; }

  double GetTime() const { return ros_img_.header.stamp.toSec(); }

  sensor_msgs::Image* GetRosImg() { return &ros_img_; }

  const sensor_msgs::Image& GetRosImg() const { return ros_img_; }

  Pixel* begin() {
    void* v = &ros_img_.data[0];
    return static_cast<Pixel*>(v);
  }

  Pixel* end() {
    void* v = &ros_img_.data[ros_img_.data.size()];
    return static_cast<Pixel*>(v);
  }

  Pixel const* begin() const {
    void const* v = &ros_img_.data[0];
    return static_cast<Pixel const*>(v);
  }

  Pixel const* end() const {
    void const* v = &ros_img_.data[ros_img_.data.size()];
    return static_cast<Pixel const*>(v);
  }
};

}  // namespace img
}  // namespace util
