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
#include "rcpputils/thread.hpp"
#include "std_msgs/msg/string.hpp"

#include "rusage_utils.hpp"
#include "sched_utils.hpp"
#include "command_line_options.hpp"
#include "burn_cpu_cycles.hpp"

/* For this example, we will be configuring the main thread scheduling policy before the node
 * creation. The thread middleware threads will inherit these settings because they are created
 * after the scheduling configuration. */

using namespace std::chrono_literals;

static ContextSwitchesCounter context_switches_counter(RUSAGE_THREAD);

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto context_switches = context_switches_counter.get();
        if (context_switches > 0L) {
          RCLCPP_WARN(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
        } else {
          RCLCPP_INFO(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
        }

        burn_cpu_cycles(200ms);

        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
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
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  auto options_reader = SchedOptionsReader();
  if (!options_reader.read_options(argc, argv)) {
    options_reader.print_usage();
    return 0;
  }
  auto options = options_reader.get_options();

  // The node is spun in a separate thread and configured after it is created
  // The spin thread settings are configured after the middleware threads
  // are created. The middleware threads will NOT inherit the scheduling settings.
  // From: https://linux.die.net/man/3/pthread_create
  // "The new thread inherits copies of the calling thread's capability sets
  // (see capabilities(7)) and CPU affinity mask (see sched_setaffinity(2))."

  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPublisher>();
  rcpputils::ThreadAttribute attr;
  attr.set_sched_policy(rcpputils::SchedPolicy::fifo);
  attr.set_priority(options.priority);
  auto spin_thread = rcpputils::Thread(
    attr,
    [&]() {
      rclcpp::spin(node);
    });


  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
