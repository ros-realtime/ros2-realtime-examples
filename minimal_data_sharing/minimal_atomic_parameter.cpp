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
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("topic", 10);

    // called on real-time thread
    auto timer_callback =
      [this]() -> void {
        // simulate some sensor reading and processing
        double sensor_data = get_sensor_data();
        auto prcessed_sensor_data = process_sensor_data(sensor_data);
        auto message = std_msgs::msg::Float64();
        message.data = prcessed_sensor_data;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.data);
        this->publisher_->publish(message);
      };

    // add the timer callback to a separate callback group
    realtime_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    timer_ = this->create_wall_timer(500ms, timer_callback, realtime_callback_group_);

    // add parameter callback to the default callback group
    gain_ = declare_parameter("gain", 1.0);
    parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // called on non-real-time thread
    auto param_callback = [this](const rclcpp::Parameter & p) {
        auto new_gain = p.as_double();
        RCLCPP_INFO(this->get_logger(), "New gain: '%lf'", new_gain);
        set_sensor_gain(new_gain);
      };
    parameter_callback_handle_ =
      parameter_event_handler_->add_parameter_callback("gain", param_callback);
  }

  rclcpp::CallbackGroup::SharedPtr get_realtime_callback_group() const
  {
    return realtime_callback_group_;
  }

  double process_sensor_data(double sensor_in) const
  {
    return sensor_in *= gain_.load();
  }

  void set_sensor_gain(double new_gain)
  {
    gain_.store(new_gain);
  }

  double get_sensor_data()
  {
    // simulate some sine-wave based input
    static double rad{2 * M_PI};
    rad += M_PI / 5;
    return std::sin(rad);
  }

private:
  // Note: atomic can generate code with mutexes in it (also platform-dependent).
  // When using atomics, always check if it is lock-free.
  static_assert(std::atomic<double>::is_always_lock_free);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std::atomic<double> gain_{1.0};
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
