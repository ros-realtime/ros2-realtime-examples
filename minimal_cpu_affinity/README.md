# minimal_cpu_affinity

These examples show how to pin the process or some threads to one or multiple CPUS.

**Note: The code shown in the examples is not necessarily real-time safe. The aim of these
examples is to show a minimal usage example for a particular concept.**

## How to run

### minimal_cpu_affinity_main_thread

For this example, we will be configuring the main thread CPU affinity. In this case, the thread 
middleware threads will not inherit these settings because they are created after the affinity
configuration.

```bash
$ ros2 run minimal_cpu_affinity minimal_cpu_affinity_main_thread
```

Use `htop` or `ps` to observe which CPU is being used by each thread

```bash
$ ps -C minimal_cpu_aff -L -o tid,comm,rtprio,cls,psr
    TID COMMAND         RTPRIO CLS PSR
   6614 minimal_cpu_aff      -  TS   1
   6615 minimal_cpu_aff      -  TS   4
   6616 gc                   -  TS   2
   6617 dq.builtins          -  TS   7
   6618 dq.user              -  TS   6
   6619 tev                  -  TS   4
   6620 recv                 -  TS   5
   6621 recvMC               -  TS   3
   6622 recvUC               -  TS   0
   6623 minimal_cpu_aff      -  TS   1
```

You can enable the processor column in `htop`. Note by default `htop` starts the CPU processor
counting at 1 instead of 0 as `ps` does.

<script id="asciicast-Fg51mx1xXW7rWFcpfXV9Xr4Of" src="https://asciinema.org/a/Fg51mx1xXW7rWFcpfXV9Xr4Of.js" async></script>

### minimal_cpu_affinity_middleware_threads

For this example, we will be configuring the main thread CPU affinity. In this case, the thread
middleware threads will inherit these settings because they are created after the affinity
configuration.

```bash
$ ros2 run minimal_cpu_affinity minimal_cpu_affinity_middleware_threads
```

Use `htop` or `ps` to observe which CPU is being used by each thread

```bash
$ ps -C minimal_cpu_aff -L -o tid,comm,rtprio,cls,psr
    TID COMMAND         RTPRIO CLS PSR
   6686 minimal_cpu_aff      -  TS   1
   6687 minimal_cpu_aff      -  TS   1
   6688 gc                   -  TS   1
   6689 dq.builtins          -  TS   1
   6690 dq.user              -  TS   1
   6691 tev                  -  TS   1
   6692 recv                 -  TS   1
   6693 recvMC               -  TS   1
   6694 recvUC               -  TS   1
   6695 minimal_cpu_aff      -  TS   1
```

<script id="asciicast-fFZNMtc7rNDs84OdSNZskexNn" src="https://asciinema.org/a/fFZNMtc7rNDs84OdSNZskexNn.js" async></script>

### minimal_cpu_affinity_mask

In this case, we pin all the thread to CPUs 2 and 3 using a CPU affinity mask.

```bash
$ ros2 run minimal_cpu_affinity minimal_cpu_affinity_mask
```

Use `htop` or `ps` to observe which CPU is being used by each thread

```bash
$ ps -C minimal_cpu_aff -L -o tid,comm,rtprio,cls,psr
    TID COMMAND         RTPRIO CLS PSR
    TID COMMAND         RTPRIO CLS PSR
   6941 minimal_cpu_aff      -  TS   2
   6942 minimal_cpu_aff      -  TS   3
   6943 gc                   -  TS   3
   6944 dq.builtins          -  TS   3
   6945 dq.user              -  TS   3
   6946 tev                  -  TS   3
   6947 recv                 -  TS   2
   6948 recvMC               -  TS   3
   6949 recvUC               -  TS   3
   6950 minimal_cpu_aff      -  TS   3
```

<script id="asciicast-f1Mi5NUvwKWuW3qXil1WzDxwN" src="https://asciinema.org/a/f1Mi5NUvwKWuW3qXil1WzDxwN.js" async></script>

## Resources

- [https://www.man7.org/linux/man-pages/man3/pthread_setaffinity_np.3.html](https://www.man7.org/linux/man-pages/man3/pthread_setaffinity_np.3.html)