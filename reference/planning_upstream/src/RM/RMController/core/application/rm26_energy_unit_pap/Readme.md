# rm26_energy_unit_pap

`rm26_energy_unit_pap` 用于按固定流程抓取/放置能量单元。节点自身提供两个
`std_srvs/srv/Trigger` 服务，内部直接加载并发布预录制机械臂轨迹，不再调用外部轨迹
Trigger service。

## 接口

- `~/store_energy_unit` (`std_srvs/srv/Trigger`)：执行放置流程。
- `~/take_energy_unit` (`std_srvs/srv/Trigger`)：执行抓取流程。
- `trajectory_topic` (`trajectory_msgs/msg/JointTrajectory`)：机械臂轨迹输出。
- `joint_state_topic` (`sensor_msgs/msg/JointState`)：读取当前机械臂关节位置，用于生成起点插值轨迹。
- `open_gripper_topic` / `close_gripper_topic` (`std_msgs/msg/Bool`)：夹爪开关命令。
- `interrupt_topic` (`std_msgs/msg/Bool`)：当前流程中收到 `true` 时中断服务回调。

## 流程

### 放置

1. 根据当前 `JointState` 生成并发布到 `place_energy_unit_trajectory_file` 起点的插值轨迹。
2. 发布 `place_energy_unit_trajectory_file` 并等待轨迹结束。
3. 发布开夹爪命令。
4. 等待 `wait_after_open_gripper_sec`。
5. 发布 `restore_trajectory_file` 并等待轨迹结束。

### 抓取

1. 发布开夹爪命令。
2. 等待 `wait_after_open_gripper_sec`。
3. 根据当前 `JointState` 生成并发布到 `move_to_energy_unit_trajectory_file` 起点的插值轨迹。
4. 发布 `move_to_energy_unit_trajectory_file` 并等待轨迹结束。
5. 发布关夹爪命令。
6. 等待 `wait_after_close_gripper_sec`。
7. 发布 `take_out_energy_unit_trajectory_file` 并等待轨迹结束。

每条轨迹的等待时间为 YAML 最后一个 `time_from_start` 加
`trajectory_completion_extra_wait_sec`。该额外等待参数必须大于 0。

插值轨迹仅在 `place_energy_unit` 和 `move_to_energy_unit` 之前生成。节点会按目标轨迹的
`joint_names` 从最近一次 `JointState` 中提取当前位置；若超时、缺失任一关节或位置数据不可用，
则跳过插值并直接执行原轨迹。插值相邻点时间间隔和每关节最大旋转角度由参数控制。

流程运行中若 `interrupt_topic` 收到 `true`，节点会尽快停止当前服务回调并返回
`success=false`，返回消息包含 `interrupted`。该中断只阻止后续流程继续执行，不额外向机械臂
控制器发送急停或取消命令。

## 轨迹格式

轨迹文件使用 `arm_trajectory_record_replay` 的 YAML 格式：

```yaml
format_version: 1
joint_names: [j0, j1]
points:
  - time_from_start: 0.0
    positions: [0.0, 0.0]
    velocities: [0.0, 0.0]  # optional
  - time_from_start: 1.0
    positions: [1.0, 0.5]
```

启动时会校验：

- 轨迹路径必须是绝对路径，且文件存在。
- `joint_names` 和 `points` 必须非空。
- 每个点必须有非负、单调不减的 `time_from_start`。
- `positions` 长度必须等于 `joint_names` 长度。
- `velocities` 可选；若存在，长度也必须等于 `joint_names` 长度。

缺少任一必需轨迹文件会导致节点启动失败。路径解析由 launch 或上层配置负责，本包不解析
package share 或相对路径。

## 参数

- `store_pipeline_service_name`
- `take_pipeline_service_name`
- `trajectory_topic`
- `joint_state_topic`
- `move_to_energy_unit_trajectory_file`
- `take_out_energy_unit_trajectory_file`
- `place_energy_unit_trajectory_file`
- `restore_trajectory_file`
- `trajectory_completion_extra_wait_sec`
- `enable_trajectory_interpolation`
- `trajectory_interpolation_time_interval_sec`
- `trajectory_interpolation_max_joint_step_rad`
- `trajectory_interpolation_joint_state_timeout_sec`
- `open_gripper_topic`
- `close_gripper_topic`
- `interrupt_topic`
- `wait_after_open_gripper_sec`
- `wait_after_close_gripper_sec`
- `gripper_command_wait_for_subscription_timeout_sec`

夹爪命令发布前会等待至少一个订阅者匹配，超时则流程失败。轨迹发布不等待订阅者匹配。

## Build

```bash
colcon build --packages-select rm26_energy_unit_pap
```
