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
ros2 service call /talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 1}}"
ros2 service call /talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 3}}"
```

Terminal 4, activate lifecycle_listener:

```bash
ros2 service call /listener/change_state lifecycle_msgs/ChangeState "{transition: {id: 1}}"
ros2 service call /listener/change_state lifecycle_msgs/ChangeState "{transition: {id: 3}}"
# deactivate after some time
ros2 service call /listener/change_state lifecycle_msgs/ChangeState "{transition: {id: 4}}"
# activate again
ros2 service call /listener/change_state lifecycle_msgs/ChangeState "{transition: {id: 3}}"
# deactivate after some time
ros2 service call /listener/change_state lifecycle_msgs/ChangeState "{transition: {id: 4}}"
```

## Resources

- http://design.ros2.org/articles/node_lifecycle.html
- https://github.com/ros2/demos/blob/master/lifecycle/README.rst
