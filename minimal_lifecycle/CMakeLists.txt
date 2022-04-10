cmake_minimum_required(VERSION 3.5)
project(minimal_lifecycle)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lifecycle_talker lifecycle_talker.cpp)
ament_target_dependencies(lifecycle_talker
    "lifecycle_msgs"
    "rclcpp_lifecycle"
    "std_msgs")

add_executable(lifecycle_listener lifecycle_listener.cpp)
ament_target_dependencies(lifecycle_listener
    "lifecycle_msgs"
    "rclcpp_lifecycle"
    "std_msgs")

install(TARGETS
  lifecycle_talker
  lifecycle_listener
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
