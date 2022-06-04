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

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <iceoryx_utils/concurrent/lockfree_queue.hpp>


using namespace std::chrono_literals;

class TrajectoryGenerator : public rclcpp::Node
{
public:
  TrajectoryGenerator()
  : Node("trajectory_generator"), count_(0)
  {
    constexpr std::size_t TRAJECTORY_SIZE = 10;
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.points.reserve(TRAJECTORY_SIZE);
        for (std::size_t i = 0; i < TRAJECTORY_SIZE; i++) {
          trajectory_msgs::msg::JointTrajectoryPoint point;
          point.positions.push_back(count_++);
          message.points.push_back(point);
        }
        RCLCPP_INFO(
          this->get_logger(), "Publish trajectory with %zu points",
          message.points.size());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  size_t count_;
};

class TrajectoryController : public rclcpp::Node
{
public:
  TrajectoryController()
  : Node("trajectory_controller")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "trajectory",
      10,
      [this](trajectory_msgs::msg::JointTrajectory::UniquePtr msg) {
        for (const auto & point : msg->points) {
          queue_.tryPush(point);
        }
        RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points ", msg->points.size());
      });

    auto timer_callback =
      [this]() -> void {
        auto msg = this->queue_.pop();
        while (msg.has_value()) {
          execute_trajectory_point(msg.value());
          msg = this->queue_.pop();
        }
      };
    realtime_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    timer_ = this->create_wall_timer(100ms, timer_callback, realtime_callback_group_);
  }

  void execute_trajectory_point(const trajectory_msgs::msg::JointTrajectoryPoint & point)
  {
    // Logging is not safe for RT but this is just a demo for message passing
    for (std::size_t joint_index = 0; joint_index < point.positions.size(); joint_index++) {
      RCLCPP_INFO(
        this->get_logger(), "Trajectory point: joint %lu, position %lf",
        joint_index, point.positions[joint_index]);
    }
  }

  rclcpp::CallbackGroup::SharedPtr get_realtime_callback_group()
  {
    return realtime_callback_group_;
  }

private:
  using JointTrajectoryPointQueue =
      iox::concurrent::LockFreeQueue<trajectory_msgs::msg::JointTrajectoryPoint, 100>;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
  JointTrajectoryPointQueue queue_;
  rclcpp::CallbackGroup::SharedPtr realtime_callback_group_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto trajectory_controller = std::make_shared<TrajectoryController>();
  auto trajectory_generator = std::make_shared<TrajectoryGenerator>();

  rclcpp::executors::StaticSingleThreadedExecutor default_callback_group_executor;
  rclcpp::executors::StaticSingleThreadedExecutor realtime_executor;

  default_callback_group_executor.add_node(trajectory_generator);
  default_callback_group_executor.add_node(trajectory_controller);
  realtime_executor.add_callback_group(
    trajectory_controller->get_realtime_callback_group(),
    trajectory_controller->get_node_base_interface());

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
