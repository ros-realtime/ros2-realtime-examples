cmake_minimum_required(VERSION 3.5)
project(minimal_deadline_qos)

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

add_executable(listener_deadline_qos listener_deadline_qos.cpp)
ament_target_dependencies(listener_deadline_qos rclcpp std_msgs)

add_executable(talker_deadline_qos talker_deadline_qos.cpp)
ament_target_dependencies(talker_deadline_qos rclcpp std_msgs)

install(TARGETS
  listener_deadline_qos
  talker_deadline_qos
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
