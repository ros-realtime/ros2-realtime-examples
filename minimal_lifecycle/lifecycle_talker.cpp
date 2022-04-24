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
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleTalker()
  : rclcpp_lifecycle::LifecycleNode("talker"), count_(0)
  {
    RCLCPP_INFO(get_logger(), "Waiting to configure the node");
  }

  bool wait_for_subscription(std::chrono::seconds timeout = 10s)
  {
    bool is_ready = false;
    std::chrono::time_point<std::chrono::steady_clock> start{std::chrono::steady_clock::now()};

    while (rclcpp::ok() && !is_ready) {
      is_ready = publisher_->get_subscription_count() > 0U;
      if (!is_ready) {
        rclcpp::sleep_for(std::chrono::milliseconds(10));
      }
      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
    }
    return true;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "on_configure() is called from state %s.", state.label().c_str());
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        if (publisher_->is_activated()) {
          auto message = std_msgs::msg::String();
          message.data = "Hello, world! " + std::to_string(this->count_++);
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
          this->publisher_->publish(message);
        }
      };
    timer_ = this->create_wall_timer(1s, timer_callback);
    timer_->cancel();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "on_activate() is called from state %s.", state.label().c_str());
    RCLCPP_INFO(get_logger(), "waiting the subscription to match");

    if (!wait_for_subscription()) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "subscription matched");
    // activate timer
    timer_->reset();
    publisher_->on_activate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    timer_->cancel();
    publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "on deactivate is called from state %s.", state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    timer_.reset();
    RCLCPP_INFO(get_logger(), "on cleanup is called from state %s.", state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    timer_.reset();
    publisher_.reset();

    RCLCPP_INFO(get_logger(), "on shutdown is called from state %s.", state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LifecycleTalker>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
