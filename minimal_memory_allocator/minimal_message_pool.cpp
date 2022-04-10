// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>

#include <std_msgs/msg/int32.hpp>

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto msg_strategy =
      std::make_shared<MessagePoolMemoryStrategy<std_msgs::msg::Int32, 1>>();
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "topic",
      10,
      [this](std_msgs::msg::Int32::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
      }, rclcpp::SubscriptionOptions(),
      msg_strategy);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
