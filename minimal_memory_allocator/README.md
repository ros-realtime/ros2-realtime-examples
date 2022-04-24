# minimal_memory_allocator

## Introduction

Current state of ROS 2 memory allocator usage:
- [https://github.com/ros2/rosidl/issues/566](https://github.com/ros2/rosidl/issues/566)
- [https://github.com/ros2/rclcpp/issues/1254](https://github.com/ros2/rclcpp/issues/1254)
- [https://github.com/ros2/rclcpp/issues/1061](https://github.com/ros2/rclcpp/issues/1061)
- [https://github.com/ros2/realtime_support/pull/103](https://github.com/ros2/realtime_support/pull/103)
- [https://github.com/ros2/rclcpp/pull/1324](https://github.com/ros2/rclcpp/pull/1324)
- [https://github.com/ros2/demos/pull/467/files](https://github.com/ros2/demos/pull/467/files)
- [https://github.com/ros2/demos/issues/292](https://github.com/ros2/demos/issues/292)
- [https://github.com/ros2/ros2/issues/590](https://github.com/ros2/ros2/issues/590)

## How to run

### minimal_custom_allocator

```bash
$ ros2 run minimal_memory_allocator minimal_custom_allocator intra
```

### minimal_custom_allocator

```bash
$ ros2 run minimal_memory_allocator minimal_custom_allocator_tlsf intra
```

### listener_message_pool

```bash
$ ros2 run minimal_memory_allocator listener_message_pool
```

## Resources

- [https://docs.ros.org/en/eloquent/Tutorials/Allocator-Template-Tutorial.html](https://docs.ros.org/en/eloquent/Tutorials/Allocator-Template-Tutorial.html)
- [Practical Memory Pool Based Allocators For Modern C++ - Misha Shalem - CppCon 2020](https://www.youtube.com/watch?v=l14Zkx5OXr4)

