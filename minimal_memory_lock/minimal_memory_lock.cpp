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

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

namespace
{
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

std::pair<std::int32_t, std::int32_t> get_new_page_faults() noexcept
{
  static struct rusage rusage_prev;
  struct rusage rusage;
  std::pair<std::int32_t, std::int32_t> page_faults;

  getrusage(RUSAGE_SELF, &rusage);
  page_faults.first = static_cast<std::int32_t>(rusage.ru_minflt - rusage_prev.ru_minflt);
  page_faults.second = static_cast<std::int32_t>(rusage.ru_majflt - rusage_prev.ru_majflt);
  rusage_prev.ru_minflt = rusage.ru_minflt;
  rusage_prev.ru_majflt = rusage.ru_majflt;

  return page_faults;
}

std::int32_t get_process_memory()
{
  struct rusage rusage;
  getrusage(RUSAGE_SELF, &rusage);
  return static_cast<std::int32_t>(rusage.ru_maxrss);
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
          this->get_logger(),
          "New minor page faults: %d, New major page faults: %d", minor, major);
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
  constexpr size_t process_max_dynamic_memory = 100 * 1024 * 1024;  // 100 MB

  bool lock_memory = false;
  if (argc > 1) {
    if (std::string(argv[1]) == "-m") {
      lock_memory = true;
    }
  }

  if (lock_memory) {
    if (configure_malloc_behavior()) {
      std::cout << "Memory locked" << std::endl;
    } else {
      return EXIT_FAILURE;
    }
    if (reserve_process_memory(process_max_dynamic_memory)) {
      std::cout << "Reserved memory: " <<
        std::to_string(process_max_dynamic_memory / (1024 * 1024)) << " MB" << std::endl;

      std::cout << "Total memory process: " <<
        std::to_string(get_process_memory() / 1024) << " MB" << std::endl;
    } else {
      return EXIT_FAILURE;
    }
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPublisher>();

  auto [minor, major] = get_new_page_faults();
  RCLCPP_INFO(
    node->get_logger(),
    "Initial minor page faults: %d, Initial major page faults: %d", minor, major);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
