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

#include "rusage_utils.hpp"
#include "sched_utils.hpp"
#include "command_line_options.hpp"
#include "burn_cpu_cycles.hpp"

/* For this example, we will be using multiple callback groups to separate callbacks, so it is
 * possible to spin them in threads with different scheduling priorities. */

using namespace std::chrono_literals;

static ContextSwitchesCounter context_switches_counter(RUSAGE_THREAD);
static ContextSwitchesCounter context_switches_counter_rt(RUSAGE_THREAD);

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_pub"), count1_(0), count2_(0)
  {
    publisher1_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback1 =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count1_++);
        this->publisher1_->publish(message);
      };
    timer1_ = this->create_wall_timer(500ms, timer_callback1);

    realtime_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    publisher2_ = this->create_publisher<std_msgs::msg::String>("topic_rt", 10);
    auto timer_callback2 =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count2_++);
        this->publisher2_->publish(message);
      };
    timer2_ = this->create_wall_timer(500ms, timer_callback2, realtime_callback_group_);
  }

  rclcpp::CallbackGroup::SharedPtr get_realtime_callback_group()
  {
    return realtime_callback_group_;
  }

private:
  rclcpp::TimerBase::SharedPtr timer1_, timer2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1_, publisher2_;
  size_t count1_, count2_;
  rclcpp::CallbackGroup::SharedPtr realtime_callback_group_;
};

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_sub")
  {
    subscription1_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr) {
        auto context_switches = context_switches_counter.get();
        if (context_switches > 0L) {
          RCLCPP_WARN(
            this->get_logger(), "[sub]    Involuntary context switches: '%lu'",
            context_switches);
        } else {
          RCLCPP_INFO(
            this->get_logger(), "[sub]    Involuntary context switches: '%lu'",
            context_switches);
        }
        burn_cpu_cycles(200ms);
      });

    realtime_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = realtime_callback_group_;
    subscription2_ = this->create_subscription<std_msgs::msg::String>(
      "topic_rt",
      10,
      [this](std_msgs::msg::String::UniquePtr) {
        auto context_switches = context_switches_counter_rt.get();
        if (context_switches > 0L) {
          RCLCPP_WARN(
            this->get_logger(), "[rt_sub] Involuntary context switches: '%lu'",
            context_switches);
        } else {
          RCLCPP_INFO(
            this->get_logger(), "[rt_sub] Involuntary context switches: '%lu'",
            context_switches);
        }
        burn_cpu_cycles(200ms);
      }, subscription_options);
  }

  rclcpp::CallbackGroup::SharedPtr get_realtime_callback_group()
  {
    return realtime_callback_group_;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
  rclcpp::CallbackGroup::SharedPtr realtime_callback_group_;
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

  rclcpp::init(argc, argv);

  auto node_pub = std::make_shared<MinimalPublisher>();
  auto node_sub = std::make_shared<MinimalSubscriber>();

  rclcpp::executors::StaticSingleThreadedExecutor default_executor;
  rclcpp::executors::StaticSingleThreadedExecutor realtime_executor;

  default_executor.add_node(node_pub);
  default_executor.add_node(node_sub);

  // the real-time callback groups are added to a specific executor which is run in a thread
  // with real-time scheduling
  // note we could have more than one executor with different scheduling policies and priorities
  realtime_executor.add_callback_group(
    node_pub->get_realtime_callback_group(), node_pub->get_node_base_interface());
  realtime_executor.add_callback_group(
    node_sub->get_realtime_callback_group(), node_sub->get_node_base_interface());

  // spin real-time tasks in a separate thread
  auto realtime_thread = std::thread(
    [&]() {
      realtime_executor.spin();
    });

  set_thread_scheduling(realtime_thread.native_handle(), options.policy, options.priority);

  default_executor.spin();
  realtime_thread.join();

  rclcpp::shutdown();
  return 0;
}
