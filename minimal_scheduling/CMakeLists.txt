cmake_minimum_required(VERSION 3.5)
project(minimal_scheduling)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(minimal_sched_fifo_main_thread minimal_sched_fifo_main_thread.cpp)
ament_target_dependencies(minimal_sched_fifo_main_thread rclcpp std_msgs)

add_executable(minimal_sched_fifo_middleware_threads minimal_sched_fifo_middleware_threads.cpp)
ament_target_dependencies(minimal_sched_fifo_middleware_threads rclcpp std_msgs)

add_executable(minimal_sched_fifo_spin_thread minimal_sched_fifo_spin_thread.cpp)
ament_target_dependencies(minimal_sched_fifo_spin_thread rclcpp std_msgs)

add_executable(minimal_sched_fifo_callback_group minimal_sched_fifo_callback_group.cpp)
ament_target_dependencies(minimal_sched_fifo_callback_group rclcpp std_msgs)

install(TARGETS
  minimal_sched_fifo_main_thread
  minimal_sched_fifo_middleware_threads
  minimal_sched_fifo_spin_thread
  minimal_sched_fifo_callback_group
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
