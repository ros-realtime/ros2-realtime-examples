cmake_minimum_required(VERSION 3.5)
project(minimal_cpu_affinity)

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

add_executable(minimal_cpu_affinity_main_thread minimal_cpu_affinity_main_thread.cpp)
ament_target_dependencies(minimal_cpu_affinity_main_thread rclcpp std_msgs)

add_executable(minimal_cpu_affinity_middleware_threads minimal_cpu_affinity_middleware_threads.cpp)
ament_target_dependencies(minimal_cpu_affinity_middleware_threads rclcpp std_msgs)

add_executable(minimal_cpu_affinity_mask minimal_cpu_affinity_mask.cpp)
ament_target_dependencies(minimal_cpu_affinity_mask rclcpp std_msgs)

install(TARGETS
  minimal_cpu_affinity_main_thread
  minimal_cpu_affinity_middleware_threads
  minimal_cpu_affinity_mask
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
