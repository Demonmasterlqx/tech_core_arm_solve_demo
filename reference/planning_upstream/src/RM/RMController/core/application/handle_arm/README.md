# handle_arm

`handle_arm` provides a toggle-controlled looping trajectory publisher for the RM arm.

## Interfaces

- Subscribes to `trigger_topic` (`std_msgs/msg/Bool`): a rising edge toggles start or stop.
- Subscribes to `joint_state_topic` (`sensor_msgs/msg/JointState`): used to build the startup interpolation.
- Publishes to `trajectory_topic` (`trajectory_msgs/msg/JointTrajectory`).

On start, the node reads the current arm joint positions, interpolates to the first point of
`trajectory_file`, then repeatedly publishes the loaded trajectory. On stop, it exits its internal
wait immediately and does not cancel commands already accepted by the controller.

## Parameters

- `trajectory_file`: required trajectory YAML path.
- `trajectory_topic`: default `/arm_control_group_controller/joint_trajectory`.
- `trigger_topic`: default `~/toggle`.
- `joint_state_topic`: default `/joint_states`.
- `trajectory_completion_extra_wait_sec`: default `0.1`.
- `interpolation_time_interval_sec`: default `0.1`.
- `interpolation_max_joint_step_rad`: default `0.1`.
- `joint_state_timeout_sec`: default `0.2`.

## Build

```bash
colcon build --packages-select handle_arm
```
