# minimal_memory_lock

## Requirements 

- Adjust permissions for memory locking see ([https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html#adjust-permissions-for-memory-locking](https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html#adjust-permissions-for-memory-locking))
 or run as root
- Install [stress-ng](https://wiki.ubuntu.com/Kernel/Reference/stress-ng)

## How to run

Without locking memory:

```bash
$ ros2 run minimal_memory_lock minimal_memory_lock
```

Locking the memory:

```bash
$ ros2 run minimal_memory_lock minimal_memory_lock --lock-memory --sleep --stacksize 100
```

You can use stress-ng to generate stress the memory and generate page faults. Run the following 
command in another terminal before running the example:

```bash
$ stress-ng --fault 0 --perf -t 1m
# for newer kernels
$ stress-ng --userfaultfd 0 --perf -t 1m
```

## Resources

- [https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/memory](https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/memory)
- [https://rt.wiki.kernel.org/index.php/Simple_memory_locking_example](https://rt.wiki.kernel.org/index.php/Simple_memory_locking_example)
- [https://rt.wiki.kernel.org/index.php/Dynamic_memory_allocation_example](https://rt.wiki.kernel.org/index.php/Dynamic_memory_allocation_example)
- [https://rt.wiki.kernel.org/index.php/Threaded_RT-application_with_memory_locking_and_stack_handling_example](https://rt.wiki.kernel.org/index.php/Threaded_RT-application_with_memory_locking_and_stack_handling_example)
- [https://rt.wiki.kernel.org/index.php/Verifying_mlockall()_effects_on_stack_memory_proof](https://rt.wiki.kernel.org/index.php/Verifying_mlockall()_effects_on_stack_memory_proof)
- [https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html#adjust-permissions-for-memory-locking](https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html#adjust-permissions-for-memory-locking)
