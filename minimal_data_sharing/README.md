# minimal_data_sharing

## How to run

### minimal_atomic_message

In this example, we use a message as a counter for some publishers publishing concurrently. A 
subcriber received the messages from the publishers and checks there are no repeated values. If 
that is the case it will log a warning message. 

In the first case, a standard message is used and the message data is incremented in a non 
tread-safe fashion. This could lead to race conditions and some repeated values might be 
observed by the subscriber.

```bash
$ ros2 run minimal_data_sharing minimal_atomic_message
```

<script id="asciicast-rMaUcBvvdnnbNWhnywsM2Yy8y" src="https://asciinema.org/a/rMaUcBvvdnnbNWhnywsM2Yy8y.js" async></script>

In the second case, using `--use-atomic` we will use an atomic message. Note, this is only
possible for very small messages, usually with just one field. For larger messages it's not
possible to define an atomic message type. The message is incremented using a CAS loop, therefore
we expect the increment to be executed atomically and the subscriber should not observe any
repeated value in this case.

```bash
$ ros2 run minimal_data_sharing minimal_atomic_message --use-atomic
```

<script id="asciicast-ZNvZ2jnrVvQy89su8vnyPd9qZ" src="https://asciinema.org/a/ZNvZ2jnrVvQy89su8vnyPd9qZ.js" async></script>

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
