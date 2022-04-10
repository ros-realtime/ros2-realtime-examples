// Copyright 2022 Carlos San Vicente
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

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LifecycleListener : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleListener()
  : rclcpp_lifecycle::LifecycleNode("listener")
  {
    RCLCPP_INFO(get_logger(), "Waiting to configure the node");
  }

  bool wait_for_publisher(std::chrono::seconds timeout = 10s)
  {
    bool is_ready = false;
    std::chrono::time_point<std::chrono::steady_clock> start{std::chrono::steady_clock::now()};

    while (rclcpp::ok() && !is_ready) {
      is_ready = subscription_->get_publisher_count() > 0U;
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
  on_configure(const rclcpp_lifecycle::State &) override
  {
    // TODO(carlos): add size as parameter
    constexpr std::size_t max_received_msgs = 10;
    received_msgs.reserve(max_received_msgs);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          // do nothing if not active
          return;
        }
        // accumulate received msgs and deactivate when we are done
        if (received_msgs.size() < max_received_msgs) {
          received_msgs.push_back(*msg);
        } else {
          this->deactivate();
        }
      });
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "on_activate() is called.");
    RCLCPP_INFO(get_logger(), "waiting the publisher to match");

    if (!wait_for_publisher()) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "publisher matched");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "on deactivate is called from state %s.", state.label().c_str());
    // log all the received messages
    std::for_each(
      received_msgs.begin(), received_msgs.end(), [this](const
      std_msgs::msg::String & msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      });
    received_msgs.clear();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    subscription_.reset();
    RCLCPP_INFO(get_logger(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "on shutdown is called from state %s.", state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::vector<std_msgs::msg::String> received_msgs;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LifecycleListener>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
