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

#include <pthread.h>

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
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  // The main thread settings are set before the middleware threads
  // are created. The middleware threads will inherit the CPU affinity settings.
  // From: https://linux.die.net/man/3/pthread_create
  // The new thread inherits copies of the calling thread's capability sets
  // (see capabilities(7)) and CPU affinity mask (see sched_setaffinity(2)).

  // Pin the main thread to CPU 1
  int cpu_id = 1;
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_id, &cpuset);
  auto ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (ret > 0) {
    std::cerr << "Couldn't set CPU affinity. Error code " << strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }
  // Check the actual affinity mask assigned to the thread
  ret = pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (ret < 0) {
    std::cerr << "Couldn't get CPU affinity. Error code " << strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Pinned CPUs:" << std::endl;
  for (int i = 0; i < CPU_SETSIZE; i++) {
    if (CPU_ISSET(i, &cpuset)) {
      std::cout << "    CPU " << std::to_string(i) << std::endl;
    }
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
