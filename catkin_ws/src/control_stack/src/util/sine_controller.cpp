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

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <chrono>
#include <string>
#include <thread>

#include "cs/util/constants.h"
#include "cs/util/twist.h"

std::pair<std::string, std::string> GetTopics(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " "
              << "/topic_to_pub /topic_to_sub" << std::endl;
    return {"", ""};
  }
  return {std::string(argv[1]), std::string(argv[2])};
}

static constexpr float kRate = 40.0;
ros::Publisher command_pub;

util::Twist Command() {
  static std::int64_t step = 0;
  const float x = std::sin(step * kPi / kRate / 2.0f);
  ++step;
  return {x, 0, 0};
}

void LaserCallback(const sensor_msgs::LaserScan&) {
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  command_pub.publish(Command().ToTwist());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "SineController");
  ros::NodeHandle n;
  const auto topic = GetTopics(argc, argv);
  if (topic.first == "") {
    return 1;
  }
  command_pub = n.advertise<geometry_msgs::Twist>(topic.first, 10);
  ros::Subscriber command_pub = n.subscribe(topic.second, 10, LaserCallback);
  ros::spin();

  return 0;
}