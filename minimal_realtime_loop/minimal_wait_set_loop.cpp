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
  using StaticWaitsetWithSub = rclcpp::StaticWaitSet<1, 0, 0, 0, 0, 0>;
  StaticWaitsetWithSub static_wait_set(
    std::array<StaticWaitsetWithSub::SubscriptionEntry, 1>{{{subscription}}},
    std::array<rclcpp::GuardCondition::SharedPtr, 0>{},
    std::array<rclcpp::TimerBase::SharedPtr, 0>{},
    std::array<rclcpp::ClientBase::SharedPtr, 0>{},
    std::array<rclcpp::ServiceBase::SharedPtr, 0>{},
    std::array<StaticWaitsetWithSub::WaitableEntry, 0>{});

  while (rclcpp::ok()) {
    auto wait_result = static_wait_set.wait(100ms);
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        if (subscription->take(msg, msg_info)) {
          RCLCPP_INFO(node->get_logger(), "I heard '%s'", msg.data.c_str());
        } else {
          RCLCPP_INFO(node->get_logger(), "No message");
        }
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      RCLCPP_INFO(node->get_logger(), "wait-set waiting failed with timeout");
    }
  }

  rclcpp::shutdown();
  return 0;
}
