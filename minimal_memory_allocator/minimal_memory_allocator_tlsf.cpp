// Copyright 2015 Open Source Robotics Foundation, Inc.
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

// Original code:
// https://github.com/ros2/demos/blob/7a15d0bf14e5765ea223873b6927be908c7acbb3/demo_nodes_cpp/src/topics/allocator_tutorial.cpp

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tlsf_cpp/tlsf.hpp"


template<typename T>
using TLSFAllocator = tlsf_heap_allocator<T>;

// Override global new and delete to count calls during execution.

static bool is_running = false;
static uint32_t global_runtime_allocs = 0;
static uint32_t global_runtime_deallocs = 0;

void * operator new(std::size_t size)
{
  if (is_running) {
    global_runtime_allocs++;
  }
  return std::malloc(size);
}

void operator delete(void * ptr, size_t size) noexcept
{
  (void)size;
  if (ptr != nullptr) {
    if (is_running) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
  }
}

void operator delete(void * ptr) noexcept
{
  if (ptr != nullptr) {
    if (is_running) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
  }
}

int main(int argc, char ** argv)
{
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  using Alloc = TLSFAllocator<void>;
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node;

  std::list<std::string> keys = {"intra", "intraprocess", "intra-process", "intra_process"};
  bool intra_process = false;

  printf(
    "This simple demo shows off a custom memory allocator to count all\n"
    "instances of new/delete in the program.  It can be run in either regular\n"
    "mode (no arguments), or in intra-process mode (by passing 'intra' as a\n"
    "command-line argument).  It will then publish a message to the\n"
    "'/allocator_tutorial' topic every 10 milliseconds until Ctrl-C is pressed.\n"
    "At that time it will print a count of the number of allocations and\n"
    "deallocations that happened during the program.\n\n");

  if (argc > 1) {
    for (auto & key : keys) {
      if (std::string(argv[1]) == key) {
        intra_process = true;
        break;
      }
    }
  }

  if (intra_process) {
    printf("Intra-process pipeline is ON.\n");

    auto options = rclcpp::NodeOptions()
      .use_intra_process_comms(true);

    node = rclcpp::Node::make_shared("allocator_example", options);
  } else {
    printf("Intra-process pipeline is OFF.\n");

    auto options = rclcpp::NodeOptions()
      .use_intra_process_comms(false);

    node = rclcpp::Node::make_shared("allocator_example", options);
  }

  uint32_t counter = 0;
  auto callback = [&counter](std_msgs::msg::UInt32::SharedPtr msg) -> void
    {
      (void)msg;
      ++counter;
    };

  // Create a custom allocator and pass the allocator to the publisher and subscriber.
  auto alloc = std::make_shared<Alloc>();
  rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;
  publisher_options.allocator = alloc;
  auto publisher = node->create_publisher<std_msgs::msg::UInt32>(
    "allocator_example", 10, publisher_options);

  rclcpp::SubscriptionOptionsWithAllocator<Alloc> subscription_options;
  subscription_options.allocator = alloc;
  auto msg_mem_strat = std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
      std_msgs::msg::UInt32, Alloc>>(alloc);
  auto subscriber = node->create_subscription<std_msgs::msg::UInt32>(
    "allocator_example", 10, callback, subscription_options, msg_mem_strat);

  // Create a MemoryStrategy, which handles the allocations made by the Executor during the
  // execution path, and inject the MemoryStrategy into the Executor.
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<Alloc>>(alloc);

  rclcpp::ExecutorOptions options;
  options.memory_strategy = memory_strategy;
  rclcpp::executors::SingleThreadedExecutor executor(options);

  // Add our node to the executor.
  executor.add_node(node);

  using MessageAllocTraits =
    rclcpp::allocator::AllocRebind<std_msgs::msg::UInt32, Alloc>;
  using MessageAlloc = MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, std_msgs::msg::UInt32>;
  using MessageUniquePtr = std::unique_ptr<std_msgs::msg::UInt32, MessageDeleter>;
  MessageDeleter message_deleter;
  MessageAlloc message_alloc = *alloc;
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_alloc);

  rclcpp::sleep_for(std::chrono::milliseconds(1));
  is_running = true;

  uint32_t i = 0;
  while (rclcpp::ok()) {
    // Create a message with the custom allocator, so that when the Executor deallocates the
    // message on the execution path, it will use the custom deallocate.
    auto ptr = MessageAllocTraits::allocate(message_alloc, 1);
    MessageAllocTraits::construct(message_alloc, ptr);
    MessageUniquePtr msg(ptr, message_deleter);
    msg->data = i;
    ++i;
    publisher->publish(std::move(msg));
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    executor.spin_some();
  }
  is_running = false;

  uint32_t final_global_allocs = global_runtime_allocs;
  uint32_t final_global_deallocs = global_runtime_deallocs;
  printf("Global new was called %u times during spin\n", final_global_allocs);
  printf("Global delete was called %u times during spin\n", final_global_deallocs);

  return 0;
}
