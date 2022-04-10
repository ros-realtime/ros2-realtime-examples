cmake_minimum_required(VERSION 3.5)
project(minimal_data_sharing)

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
find_package(iceoryx_utils REQUIRED)
find_package(apex_containers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(minimal_atomic_message minimal_atomic_message.cpp)
ament_target_dependencies(minimal_atomic_message rclcpp std_msgs)

add_executable(minimal_atomic_parameter minimal_atomic_parameter.cpp)
ament_target_dependencies(minimal_atomic_parameter rclcpp std_msgs)

add_executable(minimal_lock_free_queue_logging minimal_lock_free_queue_logging.cpp)
ament_target_dependencies(minimal_lock_free_queue_logging
    rclcpp std_msgs iceoryx_utils apex_containers)

add_executable(minimal_lock_free_queue minimal_lock_free_queue.cpp)
ament_target_dependencies(minimal_lock_free_queue rclcpp iceoryx_utils trajectory_msgs)

install(TARGETS
  minimal_atomic_message
  minimal_atomic_parameter
  minimal_lock_free_queue
  minimal_lock_free_queue_logging
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
