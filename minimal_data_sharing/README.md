# minimal_data_sharing

## How to run

### minimal_atomic_message

```bash
$ ros2 run minimal_data_sharing minimal_atomic_message
```

### minimal_atomic_parameter

```bash
$ ros2 run minimal_data_sharing minimal_atomic_parameter
```

Reset the count parameter:

```bash
ros2 param set /minimal_publisher count 0
```

### minimal_lock_free_queue

```bash
$ ros2 run minimal_data_sharing minimal_lock_free_queue
```

### minimal_lock_free_queue_logging

```bash
$ ros2 run minimal_data_sharing minimal_lock_free_queue_logging
```

## Resources

- Matthias Killat - Lock-free programming for real-time systems - Meeting C++ 2021
([video](https://www.youtube.com/watch?v=j2AgjFSFgRc))
- [Eclipse iceoryx hoofs overview](https://github.com/eclipse-iceoryx/iceoryx/tree/master/iceoryx_hoofs#concurrent)
- Real time 101 - David Rowland & Fabian Renn Giles - Meeting C++ 2019
([video](https://www.youtube.com/watch?v=ndeN983j_GQ))
