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

#include "rclcpp/rclcpp.hpp"
#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"

#include <iceoryx_utils/concurrent/lockfree_queue.hpp>

static_assert(std::atomic<double>::is_always_lock_free);

// Based on: https://github.com/ros2/demos/blob/7a15d0bf14e5765ea223873b6927be908c7acbb3/pendulum_control/include/pendulum_control/pendulum_controller.hpp

using namespace std::chrono_literals;

struct PIDProperties
{
  /// Proportional constant.
  double p = 1;
  /// Integral constant.
  double i = 0;
  /// Derivative constant.
  double d = 0;
};

/// Provides a simple PID controller for the inverted pendulum.
class PendulumController : public rclcpp::Node
{
public:
  /// Default constructor.
  /**
   * \param[in] period The update period of the controller.
   * \param[in] pid The properties of the controller.
   */
  PendulumController(std::chrono::nanoseconds period, PIDProperties pid)
  : rclcpp::Node("pendulum_controller"),
    publish_period_(period), pid_(pid)
  {
    set_command(get_setpoint());

    // Calculate the controller timestep (for discrete differentiation/integration).
    dt_ = publish_period_.count() / (1000.0 * 1000.0 * 1000.0);
    if (std::isnan(dt_) || dt_ == 0) {
      throw std::runtime_error("Invalid dt_ calculated in PendulumController constructor");
    }

    // Create real-time callbacks
    realtime_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = realtime_callback_group_;

    // Create a lambda function to invoke the controller callback when a command is received.
    auto controller_subscribe_callback =
      [this](pendulum_msgs::msg::JointState::ConstSharedPtr msg) -> void
      {
        this->on_sensor_message(msg);
      };
    sensor_sub_ = this->create_subscription<pendulum_msgs::msg::JointState>(
      "pendulum_sensor",
      10,
      controller_subscribe_callback, subscription_options);

    // Initialize the publisher for the command message.
    command_pub_ = create_publisher<pendulum_msgs::msg::JointCommand>("pendulum_command", 10);
    // Create a lambda function that will fire regularly to publish the next command message.
    auto controller_publish_callback =
      [this]()
      {
        auto command_message = pendulum_msgs::msg::JointCommand().set__position(get_command());
        this->command_pub_->publish(command_message);
      };

    controller_publisher_timer_ = this->create_wall_timer(
      period, controller_publish_callback,
      realtime_callback_group_);

    // Create non-real-time callbacks
    // Create a lambda function to accept user input to command the pendulum
    auto controller_command_callback =
      [this](pendulum_msgs::msg::JointCommand::ConstSharedPtr msg) -> void
      {
        this->on_pendulum_setpoint(msg);
      };

    setpoint_sub_ = this->create_subscription<pendulum_msgs::msg::JointCommand>(
      "pendulum_setpoint", 10, controller_command_callback);

    // Set PID parameters
    declare_parameter("p", 1.0);
    declare_parameter("i", 0.0);
    declare_parameter("d", 0.0);

    setParameterEventCallback();

    // Create a lambda function that will fire regularly to publish the next results message.
    auto logger_publish_callback =
      [this]() {
        RCLCPP_INFO(
          this->get_logger(), "Pendulum command position: %lf", get_command());
        RCLCPP_INFO(
          this->get_logger(), "Pendulum state position: %lf", get_sensor_position());
        RCLCPP_INFO(
          this->get_logger(), "Pendulum setpoint: %lf", get_setpoint());
        RCLCPP_INFO(this->get_logger(), "----------------------------");
      };

    // Add a timer to enable regular publication of results messages.
    logger_publisher_timer_ = this->create_wall_timer(1s, logger_publish_callback);
  }

  /// Calculate new command based on new sensor state and PID controller properties.
  // \param[in] msg Received sensor message.
  void on_sensor_message(pendulum_msgs::msg::JointState::ConstSharedPtr msg)
  {
    if (std::isnan(msg->position)) {
      throw std::runtime_error("Sensor value was NaN in on_sensor_message callback");
    }
    // PID controller algorithm
    double setpoint = get_setpoint();
    double error = setpoint - msg->position;
    // Proportional gain is proportional to error
    double p_gain = pid_.p * error;
    // Integral gain is proportional to the accumulation of error
    i_gain_ = pid_.i * (i_gain_ + error * dt_);
    // Differential gain is proportional to the change in error
    double d_gain = pid_.d * (error - last_error_) / dt_;
    last_error_ = error;

    // Calculate the message based on PID gains
    double new_position;
    new_position = msg->position + p_gain + i_gain_ + d_gain;
    // Enforce positional limits
    if (new_position > M_PI) {
      new_position = M_PI;
    } else if (new_position < 0) {
      new_position = 0;
    }

    if (std::isnan(new_position)) {
      throw std::runtime_error("Resulting command was NaN in on_sensor_message callback");
    }

    set_command(new_position);
    set_sensor_position(msg->position);
  }

  /// Callback when a pendulum JointCommand message is received.
  // \param[in] msg The incoming message containing the position.
  void on_pendulum_setpoint(pendulum_msgs::msg::JointCommand::ConstSharedPtr msg)
  {
    set_setpoint(msg->position);
    RCLCPP_INFO(this->get_logger(), "Pendulum set to: '%lf'", msg->position);
  }

  /// Set the properties of the PID controller (gains and desired output).
  // \param[in] properties Struct representing the desired properties.
  void set_pid_properties(const PIDProperties & properties)
  {
    // TODO(carlos): set data using a lock-free mechanism
    pid_ = properties;
  }

  /// Get the properties of the controller.
  // \return Struct representing the properties of the controller.
  const PIDProperties & get_pid_properties() const
  {
    // TODO(carlos): get data using a lock-free mechanism
    return pid_;
  }

  /// Set the commanded position of the controller.
  // \param[in] command The new commanded position (in radians).
  void set_setpoint(double command)
  {
    setpoint_.store(command);
  }

  /// Get the commanded position of the controller.
  // \return The commanded position (in radians).
  double get_setpoint() const
  {
    return setpoint_.load();
  }

  /// Get the output position of the controller.
  // \return The commanded position (in radians).
  double get_command() const
  {
    return command_.load();
  }

  /// Set the output position of the controller.
  // \param[in] command The new commanded position (in radians).
  void set_command(double command)
  {
    command_.store(command);
  }

  /// Get the sensor position.
  // \return The sensor position.
  double get_sensor_position() const
  {
    return sensor_position_.load();
  }

  /// Set the sensor position.
  // \param[in] sensor_position The new sensor position (in radians).
  void set_sensor_position(double sensor_position)
  {
    sensor_position_.store(sensor_position);
  }

  rclcpp::CallbackGroup::SharedPtr get_realtime_callback_group()
  {
    return realtime_callback_group_;
  }

private:
  // based on https://github.com/ros-controls/control_toolbox/blob/d7462e982e40bfdebddbcff8849c611caf8c8422/src/pid_ros.cpp#L281-L321
  void setParameterEventCallback()
  {
    auto on_parameter_event_callback = [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        /// @note don't use get_pid_properties, it's rt
        PIDProperties pid_prop = get_pid_properties();

        for (auto & parameter : parameters) {
          const std::string param_name = parameter.get_name();
          try {
            if (param_name == "p") {
              pid_prop.p = parameter.get_value<double>();
            } else if (param_name == "i") {
              pid_prop.i = parameter.get_value<double>();
            } else if (param_name == "d") {
              pid_prop.d = parameter.get_value<double>();
            } else {
              result.successful = false;
              result.reason = "Invalid parameter";
            }
          } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Please use the right type: " << e.what());
          }
        }

        if (result.successful) {
          /// @note don't call set_pid_properties() from inside a callback
          set_pid_properties(pid_prop);
          RCLCPP_INFO(
            this->get_logger(),
            "PID parameters set to: p=%lf, i=%lf, d=%lf",
            pid_prop.p, pid_prop.i, pid_prop.d);
        }

        return result;
      };

    parameter_callback_ = this->get_node_parameters_interface()->add_on_set_parameters_callback(
      on_parameter_event_callback);
  }

  // controller should publish less frequently than the motor
  std::chrono::nanoseconds publish_period_;
  PIDProperties pid_;

  std::atomic<double> sensor_position_{M_PI / 2};
  std::atomic<double> command_{M_PI / 2};
  std::atomic<double> setpoint_ {M_PI / 2};

  // state for PID controller
  double last_error_ = 0;
  double i_gain_ = 0;
  double dt_;

  using PIDPropertiesBuffer = iox::concurrent::LockFreeQueue<PIDProperties, 1>;
  PIDPropertiesBuffer pid_buffer_;

  rclcpp::TimerBase::SharedPtr controller_publisher_timer_;
  rclcpp::TimerBase::SharedPtr logger_publisher_timer_;
  rclcpp::Publisher<pendulum_msgs::msg::JointCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<pendulum_msgs::msg::JointState>::SharedPtr sensor_sub_;
  rclcpp::Subscription<pendulum_msgs::msg::JointCommand>::SharedPtr setpoint_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::CallbackGroup::SharedPtr realtime_callback_group_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  PIDProperties pid;
  auto node = std::make_shared<PendulumController>(std::chrono::nanoseconds(960000), pid);

  rclcpp::executors::StaticSingleThreadedExecutor default_callback_group_executor;
  rclcpp::executors::StaticSingleThreadedExecutor realtime_executor;

  default_callback_group_executor.add_node(node);
  realtime_executor.add_callback_group(
    node->get_realtime_callback_group(), node->get_node_base_interface());
  // spin real-time tasks in a separate thread
  auto realtime_thread = std::thread(
    [&]() {
      realtime_executor.spin();
    });

  default_callback_group_executor.spin();
  realtime_thread.join();

  rclcpp::shutdown();
  return 0;
}
