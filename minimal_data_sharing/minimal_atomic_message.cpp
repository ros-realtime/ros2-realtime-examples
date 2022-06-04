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

#include <rcutils/cmdline_parser.h>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

namespace
{
static constexpr const std::string_view OPTION_USE_ATOMIC = "--use-atomic";

void print_usage()
{
  printf("\nUsage:\n");
  printf(
    "%s : use atomic increment for the message data."
    " Defaults to false.\n", OPTION_USE_ATOMIC.data());
}
}  // namespace

class MinimalPublisher : public rclcpp::Node
{
public:
  explicit MinimalPublisher(bool use_atomic)
  : Node("minimal_publisher"),
    atomic_msg_(std_msgs::msg::UInt32()),
    msg_(std_msgs::msg::UInt32()),
    use_atomic_{use_atomic}
  {
    // Make the callback groups reentrant so they can run concurrently
    auto reentrant_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions options;
    options.callback_group = reentrant_callback_group;
    auto publisher = this->create_publisher<std_msgs::msg::UInt32>("topic", 10, options);
    auto timer_callback =
      [this, publisher]() -> void {
        // The message data will be incremented concurrently and then published
        std_msgs::msg::UInt32 msg;
        if (use_atomic_) {
          msg = increment_atomic_message_data();
        } else {
          msg = increment_message_data();
        }
        publisher->publish(msg);
      };
    // Crate as many publishers and CPU cores
    const auto num_pub = std::thread::hardware_concurrency();
    for (auto pub_index = 0U; pub_index < num_pub; pub_index++) {
      timers_.push_back(this->create_wall_timer(1ms, timer_callback, reentrant_callback_group));
    }
  }

  std_msgs::msg::UInt32 increment_atomic_message_data()
  {
    // we use a CAS loop to increment the data
    // Note it is not recommended to implement custom lock-free algorithms since it is
    // prone to errors. This is just for demonstration purposes only.
    auto old_value = atomic_msg_.load();
    auto new_value = old_value;
    new_value.data++;
    do {
      new_value = old_value;
      new_value.data++;
    } while(!atomic_msg_.compare_exchange_weak(old_value, new_value));
    return new_value;
  }

  std_msgs::msg::UInt32 increment_message_data()
  {
    // this increment is not thread safe, race conditions are expected
    msg_.data++;
    return msg_;
  }

private:
  using AtomicUint32Msg = std::atomic<std_msgs::msg::UInt32>;

  // Note: atomic can generate code with mutexes in it (also platform-dependent).
  // When using atomics, always check if it is lock-free.
  static_assert(AtomicUint32Msg::is_always_lock_free);

  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  AtomicUint32Msg atomic_msg_;
  std_msgs::msg::UInt32 msg_;
  bool use_atomic_;
};

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::UInt32>(
      "topic",
      10,
      [this](std_msgs::msg::UInt32::UniquePtr msg) {
        // The received message is expected to be unique if the data was incremented atomically
        // We print a warning if this is not the case
        auto search = received_values_.find(msg->data);
        if (search != received_values_.end()) {
          RCLCPP_WARN(this->get_logger(), "I heard a repeated value: '%d'", msg->data);
        }
        received_values_.insert(msg->data);
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_;
  std::set<std::uint32_t> received_values_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Argument count and usage
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return EXIT_SUCCESS;
  }

  bool use_atomic{false};
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_USE_ATOMIC.data())) {
    use_atomic = true;
  }

  std::cout << "Use atomic: " << std::to_string(use_atomic) << std::endl;

  auto publishers = std::make_shared<MinimalPublisher>(use_atomic);
  auto subscriber = std::make_shared<MinimalSubscriber>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(publishers);
  executor.add_node(subscriber);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
