# minimal_lifecycle

## How to run

Terminal 1:

```bash
$ ros2 run minimal_lifecycle lifecycle_talker
```

Terminal 2:

```bash
$ ros2 run minimal_lifecycle lifecycle_listener
```

Terminal 3, activate lifecycle_talker:

```bash
$ ros2 lifecycle set /talker configure
$ ros2 lifecycle set /talker activate
```

Terminal 4, activate lifecycle_listener:

```bash
$ ros2 lifecycle set /listener configure
$ ros2 lifecycle set /listener activate
# wait until node is deactivated
# set the max received messages param to 10
$ ros2 param set /listener max_received_msgs 10
# activate again
$ ros2 lifecycle set /listener activate
```

## Resources

- [http://design.ros2.org/articles/node_lifecycle.html](http://design.ros2.org/articles/node_lifecycle.html)
- [https://github.com/ros2/demos/blob/master/lifecycle/README.rst](https://github.com/ros2/demos/blob/master/lifecycle/README.rst)
