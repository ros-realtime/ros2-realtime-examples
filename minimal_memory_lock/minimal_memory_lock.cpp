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

#include "rcutils/cmdline_parser.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rusage_utils.hpp"
#include "memory_lock_utils.hpp"

using namespace std::chrono_literals;

static const char * OPTION_LOCK_MEMORY = "--lock-memory";
static const char * OPTION_MEMORY_PREALLOCATION = "--memory-preallocation";
static const char * OPTION_THREAD_STACK_SIZE = "--stacksize";
static const char * OPTION_NO_WAIT = "--no-wait";
static const char * OPTION_ALLOCATE_MEMORY_IN_NODE = "--allocate-in-node";

namespace
{
void print_usage()
{
  printf("\nUsage:\n");
  printf("%s : lock memory\n", OPTION_LOCK_MEMORY);
  printf("%s : preallocate memory for the process (MB)\n", OPTION_MEMORY_PREALLOCATION);
  printf("%s : set the default thread stack size (KB)\n", OPTION_THREAD_STACK_SIZE);
  printf("%s : do not wait for middleware initialization\n", OPTION_NO_WAIT);
  printf("%s : allocate dynamic in node when spinning (MB)\n", OPTION_ALLOCATE_MEMORY_IN_NODE);
}

void allocate_heap_memory(size_t size)
{
  std::unique_ptr<uint8_t[]> buffer(new uint8_t[size]);
  for (size_t i = 0; i < size; i += sysconf(_SC_PAGESIZE)) {
    buffer[i] = i;
  }
}

void wait_for_middleware_initialization()
{
  printf("Sleeping here so all the middleware threads are created\n");
  std::this_thread::sleep_for(500ms);
}
}  // namespace

class MinimalPublisher : public rclcpp::Node
{
public:
  explicit MinimalPublisher(size_t memory_to_allocate_in_node)
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this, memory_to_allocate_in_node]() -> void {
        // allocate some memory on the heap to see if we generate page faults
        if (memory_to_allocate_in_node > 0) {
          allocate_heap_memory(memory_to_allocate_in_node);
        }

        // log new page faults since last timer callback
        PageFaults delta = page_fault_counter_.get_page_faults_difference();
        if (delta.major > 0 || delta.minor > 0) {
          RCLCPP_WARN(
            this->get_logger(), "New page faults during spin: [minor: %ld, major: %ld]",
            delta.minor, delta.major);
        } else {
          RCLCPP_INFO(
            this->get_logger(), "New page faults during spin: [minor: %ld, major: %ld]",
            delta.minor, delta.major);
        }

        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

  void init_page_fault_counter()
  {
    page_fault_counter_.reset();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  PageFaultCounter page_fault_counter_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Argument count and usage
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Configuration variables
  size_t memory_pool = 0;
  size_t stack_size = 0;
  bool enable_lock_memory = false;
  bool use_wait = true;
  size_t memory_to_allocate_in_node = 0;

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LOCK_MEMORY)) {
    enable_lock_memory = true;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_MEMORY_PREALLOCATION)) {
    memory_pool = 1024 * 1024 *
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_MEMORY_PREALLOCATION));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_THREAD_STACK_SIZE)) {
    stack_size = 1024 *
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_THREAD_STACK_SIZE));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_NO_WAIT)) {
    use_wait = false;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_ALLOCATE_MEMORY_IN_NODE)) {
    memory_to_allocate_in_node = 1024 * 1024 *
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_ALLOCATE_MEMORY_IN_NODE));
  }

  // Change a new default thread stack size to reduce memory foot-print
  if (stack_size > 0) {
    set_default_thread_stacksize(stack_size);
  }

  print_process_memory("Process memory before node creation: ");

  // Create nodes
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>(memory_to_allocate_in_node);

  // Wait until the middleware is completely initialize
  if (use_wait) {
    wait_for_middleware_initialization();
  }

  if (enable_lock_memory) {
    print_process_memory("Process memory before locking: ");
    lock_memory();
    printf("Process memory locked\n");
    print_process_memory("Process memory after locking: ");

    if (memory_pool > 0) {
      printf("Total memory pool to preallocate: %ld MB\n", memory_pool / (1024 * 1024));
      auto current_process_memory = get_process_memory() * 1024;
      if (current_process_memory > memory_pool) {
        printf(
          "Process memory higher than maximum memory to reserve: : %ld > %ld\n",
          current_process_memory, memory_pool);
        return EXIT_FAILURE;
      }
      auto additional_memory_to_reserve = memory_pool - current_process_memory;
      printf("Additional memory to reserve: %ld\n", additional_memory_to_reserve / (1024 * 1024));
      preallocate_memory(additional_memory_to_reserve);
      print_process_memory("Process memory after memory pool reservation: ");
    }
  } else {
    printf("Process memory is not being locked. Memory page faults may occur.\n");
  }

  node->init_page_fault_counter();

  print_process_memory("Process memory before spin: ");
  print_page_faults("Total page faults before spin");

  rclcpp::spin(node);

  print_process_memory("Process memory after spin: ");
  print_page_faults("Total page faults after spin");

  rclcpp::shutdown();
  return 0;
}
