// Copyright 2019 Open Source Robotics Foundation, Inc.
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

// Original code:
// https://github.com/ros2/demos/blob/7a15d0bf14e5765ea223873b6927be908c7acbb3/demo_nodes_cpp/src/topics/talker_loaned_message.cpp

#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LoanedMessageListener : public rclcpp::Node
{
public:
  LoanedMessageListener()
  : Node("loaned_message_listener")
  {
    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // The executor will implicitly use the corresponding code path to handle loaned messages if
    // it is supported by the middleware
    auto pod_sub_cb =
      [this](std_msgs::msg::Float64::UniquePtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "POD subscription. I heard: '%lf'", msg->data);
      };
    auto non_pod_sub_cb =
      [this](std_msgs::msg::String::UniquePtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "Non POD subscription. I heard: '%s'", msg->data.c_str());
      };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pod_sub_ = this->create_subscription<std_msgs::msg::Float64>("chatter_pod", qos, pod_sub_cb);
    non_pod_sub_ = this->create_subscription<std_msgs::msg::String>("chatter", qos, non_pod_sub_cb);

    RCLCPP_INFO(
      this->get_logger(), "POD publisher can_loan_messages: %s",
      pod_sub_->can_loan_messages() ? "True" : "False");
    RCLCPP_INFO(
      this->get_logger(), "Non POD publisher can_loan_messages: %s",
      non_pod_sub_->can_loan_messages() ? "True" : "False");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pod_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr non_pod_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoanedMessageListener>());
  rclcpp::shutdown();
  return 0;
}
