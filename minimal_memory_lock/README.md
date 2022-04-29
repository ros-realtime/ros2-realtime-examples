# minimal_memory_lock

## Description

Page faults is a non-deterministic event and should be avoided during
real-time execution.

This example shows how to lock the process virtual memory into physical RAM 
in order to prevent page faults happening during the applications real-time 
runtime phase. 

## Requirements 

- Adjust permissions for memory locking. Add to `/etc/security/limits.conf`
(as sudo):

```bash
<your username>    -   memlock   <limit in kB>
```

## Adding some memory stress

While running the examples, you can use [stress-ng](https://wiki.ubuntu.com/Kernel/Reference/stress-ng)
to generate stress the memory and generate page faults. Run the following
command in another terminal before running the example:

```bash
$ stress-ng --fault 0 --perf -t 1m
# for newer kernels
$ stress-ng --userfaultfd 0 --perf -t 1m
```

## How to run

### No memory lock

By default, the memory is not locked, and we can see some page faults while
the node is spinning.

```bash
$ ros2 run minimal_memory_lock minimal_memory_lock
Process memory before node creation:  13 MB
Process memory is not being locked. Memory page faults may occur.
Process memory before spin:  13 MB
Total page faults before spin [Minor: 789, Major: 0]
[WARN] [1651228546.831772017] [minimal_publisher]: New page faults during spin: [minor: 3, major: 0]
[WARN] [1651228547.331783493] [minimal_publisher]: New page faults during spin: [minor: 3, major: 0]
[INFO] [1651228547.831905095] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228548.331781121] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228548.831736137] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228549.331774937] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228549.831664055] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228550.331663058] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228550.831664222] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228551.331658742] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228551.831660235] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228552.331768024] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[WARN] [1651228552.831653605] [minimal_publisher]: New page faults during spin: [minor: 1, major: 0]
[INFO] [1651228553.331652699] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651228553.831650582] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[WARN] [1651228554.331644228] [minimal_publisher]: New page faults during spin: [minor: 1, major: 0]
```

Using the `--allocate-in-node` we can simulate the node allocating some memory
on the heap when spinning.

If we allocate 50 MBytes each time we can see page faults generated more
frequently:

```bash
ros2 run minimal_memory_lock minimal_memory_lock --allocate-in-node 50
Process memory before node creation:  13 MB
Process memory is not being locked. Memory page faults may occur.
Process memory before spin:  13 MB
Total page faults before spin [Minor: 790, Major: 0]
[WARN] [1651228794.768297975] [minimal_publisher]: New page faults during spin: [minor: 12800, major: 0]
[WARN] [1651228795.268509771] [minimal_publisher]: New page faults during spin: [minor: 12803, major: 0]
[WARN] [1651228795.767921477] [minimal_publisher]: New page faults during spin: [minor: 12800, major: 0]
[WARN] [1651228796.267221048] [minimal_publisher]: New page faults during spin: [minor: 12800, major: 0]
[WARN] [1651228796.769758758] [minimal_publisher]: New page faults during spin: [minor: 12800, major: 0]
[WARN] [1651228797.271399242] [minimal_publisher]: New page faults during spin: [minor: 12800, major: 0]
[WARN] [1651228797.773109372] [minimal_publisher]: New page faults during spin: [minor: 12800, major: 0]
```

### Locking the memory

Using the `--lock-memory` option we enforce all the process virtual memory
to be locked into the physical RAM memory. Now we expect to see no page faults
in the spin phase.

```bash
$ ros2 run minimal_memory_lock minimal_memory_lock --lock-memory
Process memory before node creation:  13 MB
Process memory before locking:  13 MB
Process memory locked
Process memory after locking:  91 MB
Process memory before spin:  91 MB
Total page faults before spin [Minor: 20439, Major: 0]
[INFO] [1651229005.207436740] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229005.707370395] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229006.207337949] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229006.707390404] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229007.207372475] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229007.707527666] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
```

In the applications allocate memory dynamically during runtime we may see some
page faults the first time the memory is allocated. After that, because we
are using `MCL_FUTURE` that memory will be locked too.

```bash
ros2 run minimal_memory_lock minimal_memory_lock --lock-memory --allocate-in-node 50
Process memory before node creation:  13 MB
Sleeping here so all the middleware threads are created
Process memory before locking:  13 MB
Process memory locked
Process memory after locking:  91 MB
Process memory before spin:  91 MB
Total page faults before spin [Minor: 20444, Major: 0]
[WARN] [1651229441.463864983] [minimal_publisher]: New page faults during spin: [minor: 12805, major: 0]
[INFO] [1651229441.895453677] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229442.395400214] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229442.896404487] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229443.395399625] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
```

One option is to pre-allocate some memory, so we don't get page faults the
first time the memory is allocated in runtime. Using the option
`--memory-preallocation` we can specify the total amount we want to
pre-allocate in for our process.

In this case, we preallocate 150 MB, this is enough to lock all the memory
locked on the node construction, and the additional memory that will be
allocated in runtime.

```bash
ros2 run minimal_memory_lock minimal_memory_lock --lock-memory --allocate-in-node 50 --memory-preallocation 150
Process memory before node creation:  13 MB
Sleeping here so all the middleware threads are created
Process memory before locking:  13 MB
Process memory locked
Process memory after locking:  91 MB
Total memory pool to preallocate: 150 MB
Additional memory to reserve: 58
Process memory after memory pool reservation:  151 MB
Process memory before spin:  151 MB
Total page faults before spin [Minor: 35294, Major: 0]
[INFO] [1651229844.746268910] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229845.218119805] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229845.717945891] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229846.218008774] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651229846.717899510] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
```

### Tuning the thread stack size

In the previous examples we saw how process memory after locking is
considerable. This happens because `mlockall` will lock the memory stack of
all the threads created by the process. By default, this is 8 MB per
thread in Linux. This could be overkill if we have a high number of threads.
One way to overcome this issue is to set stack size of each thread.

For **demonstration purposes** we change the default stack size of the threads
using the `--stacksize` option.

```bash
ros2 run minimal_memory_lock minimal_memory_lock --lock-memory --stacksize 100
Process memory before node creation:  13 MB
Sleeping here so all the middleware threads are created
Process memory before locking:  13 MB
Process memory locked
Process memory after locking:  20 MB
Process memory before spin:  20 MB
Total page faults before spin [Minor: 2228, Major: 0]
[INFO] [1651230227.557519298] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651230228.054858822] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651230228.555020229] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651230229.054889267] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651230229.554856423] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
```

We reduced the total memory used by the process. However, this
configuration should be done for each thread knowing which stack size is
needed. Otherwise, we might see a stack overflow. Some middleware
implementations allow to configure the stack size of each thread.

### Wait for middleware initialization

When the node is created, the rmw middleware will create some threads. If
we start spinning the node too soon, the middleware thread might not be
created yet, and we will see some page faults happening during the spin.

You can observe this by using the `--no-sleep` option. You might have to
use some memory stress and run the example several times to observe this issue.

```bash
ros2 run minimal_memory_lock minimal_memory_lock --lock-memory --no-wait --memory-preallocation 150
Process memory before node creation:  13 MB
Process memory before locking:  13 MB
Process memory locked
Process memory after locking:  92 MB
Total memory pool to preallocate: 150 MB
Additional memory to reserve: 57
Process memory after memory pool reservation:  150 MB
Process memory before spin:  150 MB
Total page faults before spin [Minor: 35143, Major: 0]
[WARN] [1651230838.192971118] [minimal_publisher]: New page faults during spin: [minor: 66, major: 0]
[INFO] [1651230838.693119777] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651230839.193148725] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651230839.692963505] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651230840.192955358] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
[INFO] [1651230840.692954400] [minimal_publisher]: New page faults during spin: [minor: 0, major: 0]
```

## Resources

- [https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/memory](https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/memory)
- [https://rt.wiki.kernel.org/index.php/Simple_memory_locking_example](https://rt.wiki.kernel.org/index.php/Simple_memory_locking_example)
- [https://rt.wiki.kernel.org/index.php/Dynamic_memory_allocation_example](https://rt.wiki.kernel.org/index.php/Dynamic_memory_allocation_example)
- [https://rt.wiki.kernel.org/index.php/Threaded_RT-application_with_memory_locking_and_stack_handling_example](https://rt.wiki.kernel.org/index.php/Threaded_RT-application_with_memory_locking_and_stack_handling_example)
- [https://rt.wiki.kernel.org/index.php/Verifying_mlockall()_effects_on_stack_memory_proof](https://rt.wiki.kernel.org/index.php/Verifying_mlockall()_effects_on_stack_memory_proof)
- [https://docs.ros.org/en/rolling/Tutorials/Real-Time-Programming.html](https://docs.ros.org/en/rolling/Tutorials/Real-Time-Programming.html)
