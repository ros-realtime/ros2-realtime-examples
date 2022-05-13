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
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), msg_(std_msgs::msg::UInt32())
  {
    // Make the callback groups reentrant so they can run concurrently
    auto reentrant_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions options;
    options.callback_group = reentrant_callback_group;
    for (auto pub_index = 0; pub_index < 10; pub_index++) {
      auto publisher = this->create_publisher<std_msgs::msg::UInt32>("topic", 10, options);
      auto timer_callback =
        [this, pub_index, publisher]() -> void {
          auto msg = increment_message_data();
          RCLCPP_INFO(this->get_logger(), "Publisher %d: '%d'", pub_index, msg.data);
          publisher->publish(msg);
        };
      timers_.push_back(this->create_wall_timer(100ms, timer_callback, reentrant_callback_group));
    }
  }

  std_msgs::msg::UInt32 increment_message_data()
  {
    auto old_value = msg_.load();
    auto new_value = old_value;
    new_value.data++;
    do {
      new_value = old_value;
      new_value.data++;
    } while(!msg_.compare_exchange_weak(old_value, new_value));
    return new_value;
  }

private:
  using AtomicUint32Msg = std::atomic<std_msgs::msg::UInt32>;

  // Note: atomic can generate code with mutexes in it (also platform-dependent).
  // When using atomics, always check if it is lock-free.
  static_assert(AtomicUint32Msg::is_always_lock_free);

  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  AtomicUint32Msg msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
