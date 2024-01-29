// Copyright 2023
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

/* For this example, we will be using two Executors running in different threads to separate
 * callback processing, so it is possible to spin them in threads with different scheduling
 * priorities. 
 */

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

    publisher2_ = this->create_publisher<std_msgs::msg::String>("topic_rt", 10);
    auto timer_callback2 =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count2_++);
        this->publisher2_->publish(message);
      };
    timer2_ = this->create_wall_timer(500ms, timer_callback2);
  }

private:
  rclcpp::TimerBase::SharedPtr timer1_, timer2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1_, publisher2_;
  size_t count1_, count2_;
};

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(const std::string & node_name, const std::string & topic_name)
  : Node(node_name), context_switches_counter_(RUSAGE_THREAD)
  {
    subscription1_ = this->create_subscription<std_msgs::msg::String>(
      topic_name,
      10,
      [this](std_msgs::msg::String::UniquePtr) {
        auto context_switches = context_switches_counter_.get();
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
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
  ContextSwitchesCounter context_switches_counter_;
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
  auto node_sub = std::make_shared<MinimalSubscriber>("minimal_sub1", "topic");
  auto node_sub_rt = std::make_shared<MinimalSubscriber>("minimal_sub2", "topic_rt");


  rcpputils::ThreadAttribute default_attr;
  rcpputils::ThreadAttribute realtime_attr;
  realtime_attr.set_sched_policy(rcpputils::SchedPolicy::fifo);
  realtime_attr.set_priority(options.priority);

  rclcpp::executors::SingleThreadedExecutor default_executor(rclcpp::ExecutorOptions(), default_attr);
  rclcpp::executors::SingleThreadedExecutor realtime_executor(rclcpp::ExecutorOptions(), realtime_attr);

  // the publisher and non real-time subscriber are processed by default_executor
  default_executor.add_node(node_pub);
  default_executor.add_node(node_sub);

  // real-time subscriber is processed by realtime_executor.
  realtime_executor.add_node(node_sub_rt);

  // middleware threads are not configured with real-time priority
  // because scheduling configuration of main thread is not real-time.

  // Note: you can only add complete nodes to an Executor. That is:
  // If a node has multiple entities (e.g. subscribers) then the corresponding
  // callbacks are all entities processed with the same scheduling configuration
  // of the thread, in which the Executor is spinning. 

  // spin non real-time tasks in a separate thread
  auto default_executor_thread = std::thread(
    [&]() {
      default_executor.spin();
    });

  // spin real-time tasks in a separate thread
  auto realtime_executor_thread = std::thread(
    [&]() {
      realtime_executor.spin();
    });

  default_executor_thread.join();
  realtime_executor_thread.join();

  rclcpp::shutdown();
  return 0;
}
