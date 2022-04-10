// Copyright 2022 Carlos San Vicente
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("minimal_subscriber");
  auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};
  auto subscription = node->create_subscription<std_msgs::msg::String>("topic", 10, do_nothing);
  std_msgs::msg::String msg;
  rclcpp::MessageInfo msg_info;
  std::chrono::time_point<std::chrono::steady_clock> wake_up_time{std::chrono::steady_clock::now()};
  std::chrono::milliseconds period{100};

  while (rclcpp::ok()) {
    if (subscription->take(msg, msg_info)) {
      RCLCPP_INFO(node->get_logger(), "I heard '%s'", msg.data.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(), "No message");
    }

    auto now = std::chrono::steady_clock::now();
    while (wake_up_time < now) {
      wake_up_time += period;
    }
    std::this_thread::sleep_until(wake_up_time);
  }

  rclcpp::shutdown();
  return 0;
}
