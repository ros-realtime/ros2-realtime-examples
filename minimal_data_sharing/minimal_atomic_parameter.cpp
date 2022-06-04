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

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };

    realtime_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    timer_ = this->create_wall_timer(500ms, timer_callback, realtime_callback_group_);

    count_ = declare_parameter("count", 0);
    parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto param_callback = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(this->get_logger(), "Count changed to: '%ld'", p.as_int());
        count_.store(static_cast<std::uint32_t>(p.as_int()));
      };
    parameter_callback_handle_ =
      parameter_event_handler_->add_parameter_callback("count", param_callback);
  }

  rclcpp::CallbackGroup::SharedPtr get_realtime_callback_group()
  {
    return realtime_callback_group_;
  }

private:
  // Note: atomic can generate code with mutexes in it (also platform-dependent).
  // When using atomics, always check if it is lock-free.
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::atomic<std::uint32_t> count_;
  rclcpp::CallbackGroup::SharedPtr realtime_callback_group_;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::executors::StaticSingleThreadedExecutor default_callback_group_executor;
  rclcpp::executors::StaticSingleThreadedExecutor realtime_executor;

  default_callback_group_executor.add_node(node);
  realtime_executor.add_callback_group(
    node->get_realtime_callback_group(), node->get_node_base_interface());

  // spin real-time tasks in a separate thread
  // note the thread is not configured with real-time attributes in this example
  auto realtime_thread = std::thread(
    [&]() {
      realtime_executor.spin();
    });

  default_callback_group_executor.spin();
  realtime_thread.join();

  rclcpp::shutdown();
  return 0;
}
