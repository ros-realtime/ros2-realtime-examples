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

#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>

#include <stdlib.h>
#include <limits.h>
#include <malloc.h>
#include <sys/resource.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <rcutils/cmdline_parser.h>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

static const char * OPTION_LOCK_MEMORY = "--lock-memory";
static const char * OPTION_MEMORY_PREALLOCATION = "--memory-preallocation";
static const char * OPTION_THREAD_STACK_SIZE = "--stacksize";
static const char * OPTION_USE_SLEEP = "--sleep";

namespace
{
void print_usage()
{
  printf("\nUsage:\n");
  printf("%s : lock memory\n", OPTION_LOCK_MEMORY);
  printf("%s : preallocate memory for the process (MB)\n", OPTION_MEMORY_PREALLOCATION);
  printf("%s : set the default thread stack size (KB)\n", OPTION_THREAD_STACK_SIZE);
  printf("%s : use sleep between node creation and memory lock\n", OPTION_USE_SLEEP);
}

bool configure_malloc_behavior()
{
  // Lock all current and future pages
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::cerr << "mlockall failed. Error code " << strerror(errno) << std::endl;
    return false;
  }

  // Turn off malloc trimming.
  if (mallopt(M_TRIM_THRESHOLD, -1) == 0) {
    std::cerr << "mallopt for trim threshold failed. Error code " << strerror(errno) << std::endl;
    return false;
  }

  // Turn off mmap usage.
  if (mallopt(M_MMAP_MAX, 0) == 0) {
    std::cerr << "mallopt for mmap failed. Error code " << strerror(errno) << std::endl;
    mallopt(M_TRIM_THRESHOLD, 128 * 1024);
    munlockall();
    return false;
  }

  return true;
}

bool reserve_process_memory(size_t memory_size)
{
  void * buf = nullptr;
  const size_t pg_sz = sysconf(_SC_PAGESIZE);
  int res;
  res = posix_memalign(&buf, static_cast<size_t>(pg_sz), memory_size);
  if (res != 0) {
    std::cerr << "Failed to reserve memory. Error code " << strerror(errno) << std::endl;
    return false;
  }
  memset(buf, 0, memory_size);
  free(buf);
  return true;
}

std::pair<std::int32_t, std::int32_t> get_page_faults()
{
  struct rusage rusage;
  getrusage(RUSAGE_SELF, &rusage);
  return std::make_pair(rusage.ru_minflt, rusage.ru_majflt);
}

std::pair<std::int32_t, std::int32_t> get_new_page_faults()
{
  static std::pair<std::int32_t, std::int32_t> page_faults_prev;
  std::pair<std::int32_t, std::int32_t> diff_page_faults;
  auto [minor, major] = get_page_faults();
  diff_page_faults.first = static_cast<std::int32_t>(minor - page_faults_prev.first);
  diff_page_faults.second = static_cast<std::int32_t>(major - page_faults_prev.second);
  page_faults_prev.first = minor;
  page_faults_prev.second = major;
  return diff_page_faults;
}

std::size_t get_process_memory()
{
  struct rusage rusage;
  getrusage(RUSAGE_SELF, &rusage);
  return static_cast<std::size_t>(rusage.ru_maxrss);
}

void print_page_faults(const std::string & prefix)
{
  auto [minor, major] = get_page_faults();
  printf("%s [Minor: %d, Major: %d]\n", prefix.c_str(), minor, major);
}

void print_process_memory(const std::string & prefix)
{
  printf("%s %ld MB\n", prefix.c_str(), get_process_memory() / 1024);
}

void set_default_thread_stacksize(size_t stacksize)
{
  pthread_attr_t attr;
  int ret;
  ret = pthread_getattr_default_np(&attr);
  if (ret != 0) {
    std::cerr << "Failed to call pthread_getattr_default_np. Error code: " <<
      strerror(errno) << std::endl;
    return;
  }

  ret = pthread_attr_setstacksize(&attr, stacksize);
  if (ret != 0) {
    std::cerr << "Failed to call pthread_attr_setstacksize. Error code: " <<
      strerror(errno) << std::endl;
    return;
  }
  ret = pthread_setattr_default_np(&attr);
  if (ret != 0) {
    std::cerr << "Failed to call pthread_setattr_default_np. Error code: " <<
      strerror(errno) << std::endl;
    return;
  }
}

}  // namespace

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
        this->publisher_->publish(message);
        auto [minor, major] = get_new_page_faults();
        RCLCPP_INFO(
          this->get_logger(), "Page faults during spin: [minor: %d, major: %d]", minor, major);
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

  // Argument count and usage
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Configuration variables
  size_t max_process_memory = 0;
  size_t stack_size = 0;
  bool lock_memory = false;
  bool use_sleep = false;

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LOCK_MEMORY)) {
    lock_memory = true;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_MEMORY_PREALLOCATION)) {
    max_process_memory = 1024 * 1024 *
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_MEMORY_PREALLOCATION));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_THREAD_STACK_SIZE)) {
    stack_size = 1024 *
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_THREAD_STACK_SIZE));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_USE_SLEEP)) {
    use_sleep = true;
  }

  // Change a new default thread stack size to reduce memory foot-print
  if (stack_size > 0) {
    set_default_thread_stacksize(stack_size);
  }

  print_process_memory("Process memory before node creation: ");

  // Create nodes
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();

  // Sleep required to avoid some initial minor page faults
  // TODO(carlos): find out why we see minor page faults initially
  // This could be caused by middleware threads being created concurrently
  if (use_sleep) {
    std::cout << "Sleeping here" << std::endl;
    std::this_thread::sleep_for(500ms);
  }

  if (lock_memory) {
    print_process_memory("Process memory before locking: ");
    bool memory_is_locked = configure_malloc_behavior();
    if (memory_is_locked) {
      std::cout << "Memory locked" << std::endl;
    } else {
      return EXIT_FAILURE;
    }
    print_process_memory("Process memory after locking: ");
  }
  if (max_process_memory > 0) {
    std::cout << "Memory to reserve: " <<
      std::to_string(max_process_memory / (1024 * 1024)) << " MB" << std::endl;
    auto current_process_memory = get_process_memory() * 1024;
    if (current_process_memory > max_process_memory) {
      std::cout << "Process memory higher than maximum memory to reserve: " << std::endl;
      return EXIT_FAILURE;
    }
    auto additional_memory_to_reserve = max_process_memory - current_process_memory;
    bool memory_is_reserved = reserve_process_memory(additional_memory_to_reserve);
    if (!memory_is_reserved) {
      return EXIT_FAILURE;
    }
    print_process_memory("Process memory after memory pool reservation: ");
  }

  print_process_memory("Process memory before spin: ");
  print_page_faults("Total page faults before spin");
  get_new_page_faults();  // init page fault count

  rclcpp::spin(node);

  print_process_memory("Process memory after spin: ");
  print_page_faults("Total page faults after spin");

  rclcpp::shutdown();
  return 0;
}
