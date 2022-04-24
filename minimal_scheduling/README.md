# minimal_scheduling

These examples show how to configure the thread scheduling policy and priority using pthread APIs.

## Requirements 

Adjust permissions for the scheduler, see (https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html#setting-permissions-for-the-scheduler)
or run as root.

## How to run

### minimal_sched_fifo_main_thread

For this example, we will be configuring the main thread scheduling policy with a SCHED_FIFO
real-time scheduling and with real-time priority. In this case, the thread middleware threads will 
not inherit these settings because they are created after the scheduling configuration.

```bash
$ ros2 run minimal_scheduling minimal_sched_fifo_main_thread
```

Use `htop` or `ps` to observe the priorities of the threads.

### minimal_sched_fifo_middleware_threads

For this example, we will be configuring the main thread scheduling policy before the node 
creation. The thread middleware threads will inherit these settings because they are created after
the scheduling configuration.

This could be interesting when middleware communications shall not participate in the real-time
execution. Also, this could be useful with some DDS implementations which allow to configure
each thread scheduling individually, allowing to set a priority different to the main thread.

```bash
$ ros2 run minimal_scheduling minimal_sched_fifo_middleware_threads
```

Use `htop` or `ps` to observe the priorities of the threads.

### minimal_sched_fifo_spin_thread

For this example, we will be configuring the scheduling policy of a thread used to spin the 
executor. In this case, the thread middleware threads will not inherit these settings because
the settings apply only to the spinner thread.

```bash
$ ros2 run minimal_scheduling minimal_sched_fifo_spin_thread
```

Use `htop` or `ps` to observe the priorities of the threads.

### minimal_sched_fifo_callback_group

For this example, we will be using multiple callback groups to separate callbacks, so it is
possible to spin them in threads with different scheduling priorities.

```bash
$ ros2 run minimal_scheduling minimal_sched_fifo_callback_group
```

Use `htop` or `ps` to observe the priorities of the threads.

## Resources

- [https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/sched_policy_prio/start](https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/sched_policy_prio/start)
- [https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base](https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base)
- [https://linux.die.net/man/3/pthread_setschedparam](https://linux.die.net/man/3/pthread_setschedparam)
- [https://linux.die.net/man/3/pthread_create](https://linux.die.net/man/3/pthread_create)
- Callback groups executor - Ralph Lange [video](https://www.youtube.com/watch?v=5Sd5bvvZeb0
), [slides](https://www.apex.ai/_files/ugd/984e93_f3791ae0711042a883bfc40f827d6268.pdf)
- [https://github.com/ros2/examples/tree/master/rclcpp/executors/cbg_executor](https://github.com/ros2/examples/tree/master/rclcpp/executors/cbg_executor)
- [https://discourse.ros.org/t/ros2-generated-child-thread-scheduling-policy-affects-timers/14212](https://discourse.ros.org/t/ros2-generated-child-thread-scheduling-policy-affects-timers/14212)
- [https://discourse.ros.org/t/experiment-to-inhibit-dds-and-ros2-child-threads/14313](https://discourse.ros.org/t/experiment-to-inhibit-dds-and-ros2-child-threads/14313)
