# minimal_realtime_loop

## How to run

### minimal_executor_loop

Terminal 1:

```bash
$ ros2 run minimal_realtime_loop minimal_executor_loop
```

### minimal_wait_set_loop

Terminal 1:

```bash
$ ros2 run minimal_realtime_loop minimal_wait_set_loop
```

Terminal 2:

```bash
$ ros2 run minimal_realtime_loop minimal_executor_loop
```

### minimal_polling_subscription_loop

Terminal 1:

```bash
$ ros2 run minimal_realtime_loop minimal_polling_subscription_loop
```

Terminal 2:

```bash
$ ros2 run minimal_realtime_loop minimal_executor_loop
```

## Resources

- https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/cyclic
- https://github.com/ros2/examples/tree/master/rclcpp/wait_set
