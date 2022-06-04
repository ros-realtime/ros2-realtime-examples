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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "iceoryx_utils/concurrent/lockfree_queue.hpp"
#include "string/string.hpp"

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  using LogMessage = apex::string256_t;
  using LogMessageQueue = iox::concurrent::LockFreeQueue<LogMessage, 100>;

  MinimalSubscriber()
  : Node("minimal_subscriber"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        logger_queue_.push(apex::varargs_to_string("Publishing: ", message.data.c_str()));
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        logger_queue_.push(apex::varargs_to_string("I heard: ", msg->data.c_str()));
      });

    // Note this is not an example of how to implement a real-time logger
    // This only shows how pass data from a real-time thread to another
    // thread to handle the data in a non real-time context
    logger_thread_ = std::thread(
      [this]() {
        while (rclcpp::ok()) {
          auto log_msg = logger_queue_.pop();
          while (log_msg.has_value()) {
            log_message(log_msg.value());
            log_msg = logger_queue_.pop();
          }
          rclcpp::sleep_for(100ms);
        }
      }
    );
  }

  void log_message(const LogMessage & message)
  {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  }

  ~MinimalSubscriber()
  {
    if (logger_thread_.joinable()) {
      logger_thread_.join();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  LogMessageQueue logger_queue_;
  std::thread logger_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
