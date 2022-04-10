# minimal_scheduling

## Requirements 

Adjust permissions for the scheduler, see (https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html#setting-permissions-for-the-scheduler)
or run as root.

## How to run

### minimal_sched_fifo

```bash
$ ros2 run minimal_scheduling minimal_sched_fifo
```

### minimal_sched_fifo_middleware_threads

```bash
$ ros2 run minimal_scheduling minimal_sched_fifo_middleware_threads
```

### minimal_sched_fifo_spinner_thread

```bash
$ ros2 run minimal_scheduling minimal_sched_fifo_spinner_thread
```

### minimal_sched_fifo_callback_group

```bash
$ ros2 run minimal_scheduling minimal_sched_fifo_callback_group
```

## Resources

- https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/sched_policy_prio/start
- https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base
- https://linux.die.net/man/3/pthread_setschedparam
- https://linux.die.net/man/3/pthread_create
- Callback groups executor - Ralph Lange [video](https://www.youtube.com/watch?v=5Sd5bvvZeb0
), [slides](https://www.apex.ai/_files/ugd/984e93_f3791ae0711042a883bfc40f827d6268.pdf)
- https://github.com/ros2/examples/tree/master/rclcpp/executors/cbg_executor
