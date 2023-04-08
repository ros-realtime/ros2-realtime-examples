# minimal_scheduling

These examples show how to configure the thread scheduling policy and priority using
[pthread APIs](https://linux.die.net/man/7/pthreads).

In order to show the effects of using a real-time scheduling policy, the examples show
the number of involuntary context switches in each callback execution. An involuntary context 
switches happens when the kernel scheduler preempts a task to schedule a higher priority thread
or by a I/O request.

Using a real-time scheduling policy and using a real-time priority removes or reduces the
number of involuntary context switches.

**Note: The code shown in the examples is not necessarily real-time safe. The aim of these
examples is to show a minimal usage example for a particular concept.**

## Requirements 

Adjust permissions for the scheduler. Add to `/etc/security/limits.conf` (as sudo):

```bash
<your username>    -   rtprio   98
```

## How to run

### minimal_scheduling_main_thread

For this example, we will be configuring the main thread scheduling policy of the main thread,
where the node is spinning. In this case, the middleware threads will not inherit these settings
because they are created after the scheduling configuration.

#### Using SCHED_OTHER scheduling policy

We start with the default settings, which means we don't configure the main thread scheduling
policy and priority. In Linux, by default, the `SCHED_OTHER` scheduling policy is used. Since,
this is not a real-time scheduling policy the main thread might be preempted frequently. For this 
reason, we expect to see a high number of involuntary context switches.

```bash
$ ros2 run minimal_scheduling minimal_scheduling_main_thread
```

<script id="asciicast-gxZvQuyvNZQR1Y75nFUCq7GQ8" src="https://asciinema.org/a/gxZvQuyvNZQR1Y75nFUCq7GQ8.js" async></script>

#### Using SCHED_FIFO scheduling policy

Now we run the same example but using the `SCHED_FIFO` scheduling policy and setting a priority
of 80.

There are just a few kernel threads running with a higher priority, so we expect to see a few
number of involuntary context switches or none.

```bash
$ ros2 run minimal_scheduling minimal_scheduling_main_thread --sched SCHED_FIFO --priority 80
```

Using `htop` or `ps` we can observe the priorities and the scheduling policies of the threads.

```bash
$ ps -C minimal_schedul -L -o tid,comm,rtprio,cls,psr
    TID COMMAND         RTPRIO CLS PSR
   6775 minimal_sched_f     80  FF   7
   6776 minimal_sched_f      -  TS   1
   6777 gc                   -  TS   1
   6778 dq.builtins          -  TS   0
   6779 dq.user              -  TS   6
   6780 tev                  -  TS   2
   6781 recv                 -  TS   4
   6782 recvMC               -  TS   0
   6783 recvUC               -  TS   1
   6784 minimal_sched_f      -  TS   0
```

Note we are not configuring the threads created by the RMW middleware. These threads will run
with normal priority. This option might be preferred when middleware communications shall not
participate in the real-time execution. Also, some DDS implementations allow to configure
each thread scheduling individually, allowing to set a priority different creator thread.

<script id="asciicast-XzlFpNwZKw0hea9vhfKFCMgj4" src="https://asciinema.org/a/XzlFpNwZKw0hea9vhfKFCMgj4.js" async></script>

#### Using SCHED_RR scheduling policy

Now we run the same example but using the `SCHED_RR` scheduling policy and setting a priority
of 80. We expect to see the same results as in the previous case.

```bash
$ ros2 run minimal_scheduling minimal_scheduling_main_thread --sched SCHED_RR --priority 80
```

Using `htop` or `ps` we can observe the priorities and the scheduling policies of the threads.

```bash
$ ps -C minimal_schedul -L -o tid,comm,rtprio,cls,psr
    TID COMMAND         RTPRIO CLS PSR
  16359 minimal_schedul     80  RR   3
  16360 minimal_schedul      -  TS   6
  16361 gc                   -  TS   1
  16362 dq.builtins          -  TS   6
  16363 dq.user              -  TS   1
  16364 tev                  -  TS   5
  16365 recv                 -  TS   7
  16366 recvMC               -  TS   0
  16367 recvUC               -  TS   7
  16368 minimal_schedul      -  TS   0
```

<script id="asciicast-OOGN7OVgLkOE4b3ifTPdx6DWr" src="https://asciinema.org/a/OOGN7OVgLkOE4b3ifTPdx6DWr.js" async></script>

### minimal_scheduling_middleware_threads

For this example, we will be configuring the main thread scheduling policy before the node
creation. The middleware threads will inherit these settings because they are created after
the scheduling configuration.

```bash
$ ros2 run minimal_scheduling minimal_scheduling_middleware_threads --sched SCHED_FIFO --priority 80
```

Using `htop` or `ps` we can observe the priorities and the scheduling policies of the threads.

```bash
$ ps -C minimal_schedul -L -o tid,comm,rtprio,cls,psr
    TID COMMAND         RTPRIO CLS PSR
   6794 minimal_sched_f     80  FF   2
   6795 minimal_sched_f     80  FF   6
   6796 gc                  80  FF   6
   6797 dq.builtins         80  FF   3
   6798 dq.user             80  FF   6
   6799 tev                 80  FF   6
   6800 recv                80  FF   7
   6801 recvMC              80  FF   7
   6802 recvUC              80  FF   6
   6803 minimal_sched_f     80  FF   7
```

This is similar to configuring the process real-time atrributes using the Linux [`chrt`](https://www.man7.org/linux/man-pages/man1/chrt.1.html)
tool.

<script id="asciicast-WtoeazuOds4CCpBDA0Mfkg8Ly" src="https://asciinema.org/a/WtoeazuOds4CCpBDA0Mfkg8Ly.js" async></script>

### minimal_scheduling_spin_thread

For this example, we will be configuring the scheduling policy of a thread used to spin the
executor. In this case, the middleware threads will not inherit these settings because
the settings apply only to the executor thread.

```bash
$ ros2 run minimal_scheduling minimal_scheduling_spin_thread --sched SCHED_FIFO --priority 80
```

Using `htop` or `ps` we can observe the priorities and the scheduling policies of the threads.

```bash
$ ps -C minimal_schedul -L -o tid,comm,rtprio,cls,psr
    TID COMMAND         RTPRIO CLS PSR
   6822 minimal_sched_f      -  TS   1
   6823 minimal_sched_f      -  TS   7
   6824 gc                   -  TS   5
   6825 dq.builtins          -  TS   7
   6826 dq.user              -  TS   2
   6827 tev                  -  TS   0
   6828 recv                 -  TS   2
   6829 recvMC               -  TS   0
   6830 recvUC               -  TS   7
   6831 minimal_sched_f      -  TS   0
   6832 minimal_sched_f     80  FF   7
```

<script id="asciicast-nLpeLDCmtwT5JqH9QYvVPGVXT" src="https://asciinema.org/a/nLpeLDCmtwT5JqH9QYvVPGVXT.js" async></script>

### minimal_scheduling_callback_group

For this example, we will be using multiple callback groups to separate callbacks, so it is
possible to execute them by threads with different scheduling priorities.

We run the example using `SCHED_FIFO` and priority 80 for the thread running the real-time
callbacks. The other callbacks run with normal priority. We expect to see a high number of
involuntary context switches logged only from the non-real-time subscription callbacks.

```bash
$ ros2 run minimal_scheduling minimal_scheduling_callback_group --sched SCHED_FIFO --priority 80
```

Using `htop` or `ps` we can observe the priorities and the scheduling policies of the threads.

```bash
$ ps -C minimal_schedul -L -o tid,comm,rtprio,cls,psr
    TID COMMAND         RTPRIO CLS PSR
   6847 minimal_sched_f      -  TS   1
   6848 minimal_sched_f      -  TS   0
   6849 gc                   -  TS   1
   6850 dq.builtins          -  TS   1
   6851 dq.user              -  TS   7
   6852 tev                  -  TS   1
   6853 recv                 -  TS   3
   6854 recvMC               -  TS   0
   6855 recvUC               -  TS   7
   6856 minimal_sched_f      -  TS   0
   6857 minimal_sched_f     80  FF   0
```

<script id="asciicast-FncZQ5gSgp6sNIxHj4weDra3n" src="https://asciinema.org/a/FncZQ5gSgp6sNIxHj4weDra3n.js" async></script>


### minimal_scheduling_real_time_tutorial

Example to demonstrate mixed system, with a real-time thread and non real-time thread. Two publisher callbacks are created. The first one is executed in the real-time thread, the other one is executed in a non real-time thread. Output: No preemptions of the real-time publisher and multiple preemptions of the non real-time publisher. 

```bash
$ ros2 run minimal_scheduling minimal_scheduling_real_time_tutorial
```

Output: real-time subscriber is not preempted by other kernel processes, but normal subscriber is.
```bash
[WARN] [1680947876.099416572] [minimal_sub1]: [sub]    Involuntary context switches: '8'
[INFO] [1680947876.099471567] [minimal_sub2]: [sub]    Involuntary context switches: '0'
[WARN] [1680947876.599197932] [minimal_sub1]: [sub]    Involuntary context switches: '49'
[INFO] [1680947876.599202498] [minimal_sub2]: [sub]    Involuntary context switches: '0'
[WARN] [1680947877.101378852] [minimal_sub1]: [sub]    Involuntary context switches: '25'
[INFO] [1680947877.101372018] [minimal_sub2]: [sub]    Involuntary context switches: '0'
```

## Resources

- [https://www.man7.org/linux/man-pages/man7/sched.7.html](https://www.man7.org/linux/man-pages/man7/sched.7.html)
- [https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/sched_policy_prio/start](https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/sched_policy_prio/start)
- [https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base](https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base)
- [https://linux.die.net/man/3/pthread_setschedparam](https://linux.die.net/man/3/pthread_setschedparam)
- [https://linux.die.net/man/3/pthread_create](https://linux.die.net/man/3/pthread_create)
- [https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/6/html/performance_tuning_guide/s-cpu-scheduler](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/6/html/performance_tuning_guide/s-cpu-scheduler)
- Callback groups executor - Ralph Lange [video](https://www.youtube.com/watch?v=5Sd5bvvZeb0), [slides](https://www.apex.ai/_files/ugd/984e93_f3791ae0711042a883bfc40f827d6268.pdf)
- [https://github.com/ros2/examples/tree/master/rclcpp/executors/cbg_executor](https://github.com/ros2/examples/tree/master/rclcpp/executors/cbg_executor)
- [Basic investigation of priority of DDS-generated threads.](https://discourse.ros.org/uploads/short-url/p2fAAbrKHkrSqZ9oJkZNwOOf2Hi.pdf)
