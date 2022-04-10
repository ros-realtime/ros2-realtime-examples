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


#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "osrf_testing_tools_cpp/memory_tools/memory_tools.hpp"
#include "apex_test_tools/apex_test_tools.hpp"

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>

#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

namespace
{
void function_under_test(bool allocate = false)
{
  if (allocate) {
    auto msg = std::make_shared<std_msgs::msg::Int32>();
  }
}
}  // namespace

class TestNodeMemory : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    (void)rclcpp::shutdown();
  }

  void SetUp() override
  {
    auto qos = rclcpp::QoS(10).transient_local();
    node = std::make_shared<rclcpp::Node>("my_node");
    publisher = node->create_publisher<std_msgs::msg::Int32>("topic", qos);
    auto sub_cb = [this](std_msgs::msg::Int32::UniquePtr) {message_count++;};
    auto msg_strategy =
      std::make_shared<MessagePoolMemoryStrategy<std_msgs::msg::Int32, 1>>();
    subscription = node->create_subscription<std_msgs::msg::Int32>(
      "topic",
      qos,
      sub_cb,
      rclcpp::SubscriptionOptions(),
      msg_strategy);
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription;
  std::atomic_uint32_t message_count{0};
};


TEST_F(TestNodeMemory, minimal_apex_test_tools) {
  // change allocate to true to see the allocation backtrace
  bool allocate = false;
  apex_test_tools::memory_test::start();
  function_under_test(allocate);
  apex_test_tools::memory_test::stop();
}

TEST_F(TestNodeMemory, minimal_osrf_testing_tools_cpp_memory_tools) {
  // change allocate to true to see the allocation backtrace
  bool allocate = false;
  osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
  function_under_test(allocate);
  osrf_testing_tools_cpp::memory_tools::expect_no_malloc_end();
}

// TODO(carlos): add tests for rclcpp publish, take and spin functions
