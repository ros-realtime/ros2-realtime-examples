cmake_minimum_required(VERSION 3.5)
project(minimal_realtime_loop)

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

add_executable(minimal_wait_set_loop minimal_wait_set_loop.cpp)
ament_target_dependencies(minimal_wait_set_loop rclcpp std_msgs)

add_executable(minimal_polling_subscription_loop minimal_polling_subscription_loop.cpp)
ament_target_dependencies(minimal_polling_subscription_loop rclcpp std_msgs)

add_executable(minimal_executor_loop minimal_executor_loop.cpp)
ament_target_dependencies(minimal_executor_loop rclcpp std_msgs)

install(TARGETS
  minimal_wait_set_loop
  minimal_polling_subscription_loop
  minimal_executor_loop
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
