# minimal_loaned_messages

## Requirements 

This requires a middleware implementation with zero-copy support.

To use iceroyx, install rmw_iceoryx https://github.com/ros2/rmw_iceoryx

## How to run

### Using iceroyx 

Run roudi in terminal 1:

```bash
/opt/ros/rolling/bin/iox-roudi
```

Terminal 2:

```bash
$ RMW_IMPLEMENTATION=rmw_iceoryx_cpp ros2 run minimal_loaned_messages loaned_message_talker
```

Terminal 3:

```bash
$ RMW_IMPLEMENTATION=rmw_iceoryx_cpp ros2 run minimal_loaned_messages loaned_message_listener
```

## Resources

- http://design.ros2.org/articles/zero_copy.html
- ROSCon 2019 Macau: A true zero copy RMW implementation for ROS2 ([video](https://vimeo.com/379127778), 
[slides](https://roscon.ros.org/2019/talks/roscon2019_truezerocopy.pdf))
- 
