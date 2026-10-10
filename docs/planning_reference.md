# 三级兑换规划：参考源码与复刻边界

本次参考固定为 HKUSTGZ-ROBOMASTER-PNX 的 [UpperComputerControlOpenSourceDoc](https://github.com/HKUSTGZ-ROBOMASTER-PNX/UpperComputerControlOpenSourceDoc/tree/6edc630a1a14a3904b42d5a360e9020562eefc99)，commit `6edc630a1a14a3904b42d5a360e9020562eefc99`。以下结论来自固定源码审计，不表示已运行上游 ROS 2/Pinocchio 包，也不代表真实机械臂精度。

**参考的核心是工具方向约束的任务雅可比 IK，以及沿已给定轨迹逐点求解。它没有实现三级兑换路径生成、避障 RRT、速度/加速度受限的时间参数化。** 三级任务采样、碰撞场景、时间约束和基准计时均须明确标为本项目扩展。

## 1. 快照与许可证

[source_manifest.json](../reference/planning_upstream/source_manifest.json) 保存每个文件的原始路径、固定 SHA raw URL、字节数、SHA-256 和 Git blob SHA-1。选择性归档包括完整 `axis_free_rotate_ik` 包、完整 `handle_arm` 包、相关机械臂报告，以及 v3 MoveIt launch / joint-limit 配置；没有下载整个上游仓库、模型权重或运行缓存。上游 tree API 返回 `truncated=false`。

离线验证，以及仅恢复缺失的固定版本文件：

```bash
python reference/planning_upstream/fetch_sources.py
python reference/planning_upstream/fetch_sources.py --download
```

下载逐文件校验后原子替换，`.part` 在退出时清理；已有文件若不匹配会直接报错，不覆盖本地修改。

[LICENSE_NOTICE.md](../reference/planning_upstream/LICENSE_NOTICE.md) 保留归属和缺失事实：IK 包 `package.xml:8` 的原文是 `TODO: License declaration`；`handle_arm/package.xml:8` 声明 `MIT`。在完整上游树中没有仓库根目录或这两个包的独立许可证正文，仅 `ecat_interface` 及其依赖有其他许可证。不能把无关子模块的许可证或本地机械臂模型包的 BSD 声明套到 IK 上。

以下路径简写均相对于快照内的 `src/RM/RMController/`：

| 简写 | 固定源码 |
| --- | --- |
| `IK.hpp` | [submodule_lib/axis_free_rotate_ik/include/axis_free_rotate_ik/axis_free_rotate_ik.hpp](../reference/planning_upstream/src/RM/RMController/submodule_lib/axis_free_rotate_ik/include/axis_free_rotate_ik/axis_free_rotate_ik.hpp) |
| `IK.cpp` | [submodule_lib/axis_free_rotate_ik/src/axis_free_rotate_ik.cpp](../reference/planning_upstream/src/RM/RMController/submodule_lib/axis_free_rotate_ik/src/axis_free_rotate_ik.cpp) |
| `ROS.cpp` | [submodule_lib/axis_free_rotate_ik/src/axis_free_rotate_ik_ros.cpp](../reference/planning_upstream/src/RM/RMController/submodule_lib/axis_free_rotate_ik/src/axis_free_rotate_ik_ros.cpp) |
| `theory` | [submodule_lib/axis_free_rotate_ik/theory/IK逆解.md](../reference/planning_upstream/src/RM/RMController/submodule_lib/axis_free_rotate_ik/theory/IK逆解.md) |
| `loop.cpp` | [core/application/handle_arm/src/trajectory_loop_node.cpp](../reference/planning_upstream/src/RM/RMController/core/application/handle_arm/src/trajectory_loop_node.cpp) |

## 2. 真实自由轴任务与坐标契约

输入是机械臂基座系中的目标工具位置 `desired_position`，同系中的目标方向 `desired_direction`，以及工具局部轴 `desired_up_vector`。后者名称虽有 “up”，其实际语义是工具局部轴，不是基座系向上向量。两轴都在求解时归一化。逐点 API 中局部轴叫 `local_axis`，默认工具 Z 轴。

设 `T_BE(q)=(R(q),p(q))`，`a=R(q)a0`，`ad` 为目标单位轴：

```text
ep = pd - p(q)
ea = a × ad
Pa = I - a aᵀ
Jtask = [Jv; Pa Jω]
e = [ep; ea]
Δq = Jtaskᵀ (Jtask Jtaskᵀ + λ² I6)⁻¹ e
Δq = per_joint_clip(α Δq, -delta_max_q, +delta_max_q)
q_next = clamp_to_URDF_limits(integrate(q, Δq))
```

`IK.cpp:732–832` 实现上述任务，`346–356` 用固定 6×6 normal matrix 的 LDLT 求解，不显式求逆。`Pa` 使用**当前工具轴 a**。矩阵保留六行，但轴投影只提供两个独立姿态约束，普通非奇异情况下总任务秩为五；绕该工具轴的自转不受约束。不能用普通完整六维姿态 IK，或随意删去世界 Z 角速度，声称等价复刻。

`IK.cpp:327–343` 从 Pinocchio `LOCAL_WORLD_ALIGNED` 末端 Jacobian 出发，将线速度、角速度两块分别旋转到基座系。末端位置来自 `oMf[base]⁻¹ oMf[end]`。雅可比与误差必须同系，线速度部分是末端点速度；不要误用绕世界原点的空间 twist 平移部分。

`theory` 给出可选权重 `wp, wa`、自适应阻尼公式及零空间目标的背景，但**实际自由轴源码没有 wp/wa 参数、没有自适应 λ、没有零空间项**。代码等价于两个权重为一，直接混合米和弧度。自适应阻尼、关节中心化、随机重启、线搜索、SVD 截断均属于另行方法，不能偷偷加到基线。

### 收敛、反向轴与关节模型

- 停止条件：`||ep||² <= position_tolerance²` 且 `dot(a,ad) >= cos(axis_tolerance)`；角误差输出为 `acos(clamp(dot,-1,1))`。角度容差达到 π 时使用 dot 下界 -1。
- 近反向特判：dot `< -1+1e-6` 且叉乘范数平方 `< 1e-12`，改用 `π * stable_perpendicular_axis(a)`。该垂直方向由 `a×X` 给出；若 `|a·X|>0.9` 则改用 `a×Y`，随后归一化。以叉乘范数作为最终成功标准会误收 180° 反向，应避免。
- 缺省初值为 Pinocchio neutral，有显式初值时先作位置限位裁剪。`delta_max_q` 是**一次迭代**增量上界，不是采样点间跳变上界，也不是物理速度。
- 构造器从指定 base 到 end 提取链，其他关节在 neutral 锁定。仅支持每关节 `nq=nv=1` 的链；Pinocchio 的连续转动关节等非此配置会拒绝。实际 RM26 使用有界七转动关节，不能将关节数七误说成 API 硬编码限制。
- 只 clamp 有效且有限的配置上下界，不包含任何碰撞、速度、加速度、力矩或抓取约束。构造器的 `/* const collision_check */` 仅为注释。
- 单点内部在耗尽迭代时返回最后更新的 q，但诊断误差来自最后更新前一次 FK；`solveTrajectory` 失败路径会重新计算最终 q 的误差。严格对照时须区分成功判定和失败诊断，独立验证必须重新 FK。

### 默认数值参数

| 参数 | 上游 `IKRequest` / `IKTrajectoryRequest` 默认 | 语义 |
| --- | --- | --- |
| `max_iterations` | 300 | 每个点最多迭代次数 |
| `tolerance` | 1e-4 | 分离容差未提供时的位置/轴角 fallback |
| `position_tolerance` | 未设置 | 米；本地基线显式采用 1e-4 m |
| `axis_tolerance` | 未设置 | 弧度；本地基线显式采用 1e-4 rad |
| `damping` | 1e-4 | λ，normal matrix 增加的是 λ² |
| `alpha` | 0.2 | DLS 后乘的步长，允许区间 (0,1] |
| `delta_max_q` | 未设置 | 可选逐关节迭代步长裁剪；默认无裁剪 |
| `max_joint_step_between_points` | 未设置 | 可选采样点间位形增量限额；默认不检查 |

所有传入标量/向量有有限性、正值或非负性、尺寸检查；方向不得为零。`solve()` 的固定姿态分支使用 `log3(Rd Rᵀ)` 和完整 Jacobian，与自由轴分支区分。完整矩阵目标不能与自由轴方向/局部轴同时提供。

## 3. 上游“轨迹规划”具体做什么

### `solveTrajectory`

`IK.cpp:372–553` 收到已给定的 `IKTrajectoryPoint[]`，先验证**全部**输入，再按顺序逐点求解。第一点用请求初值，之后严格使用上一成功点的解；每点可覆盖迭代次数、位置/轴角容差、damping、alpha、delta_max_q。没有 Cartesian 插点、路径搜索或时间参数。

若设置 `max_joint_step_between_points`，比较 `abs(pinocchio::difference(q_previous,q_current))`，超界即失败；不会缩步重试。该检查不覆盖请求 initial_guess 到第一点的连线。首个失败发生后立即终止：C++ 返回已完成点和失败点、`first_failed_index`、误差与消息；预验证失败返回空 points。不能把仅完成前缀报告为完整三级成功。

### ROS wrapper 的时间戳

`ROS.cpp:428–446` 对成功的离散 q 列表构建消息：

```text
point[i].positions = q[i]
point[i].velocities = 全零
point[i].accelerations = 空
point[i].time_from_start = (i+1) * point_time_interval
```

间隔由服务请求提供且必须正、有限；没有根据距离或运动限值调节。局部轴由请求给出，PoseStamped 经 TF 转到 base 后取 `desired_direction=R_target*local_axis`。整个 C++ 轨迹成功才返回/可选发布；失败不发布部分轨迹。服务响应是 trajectory/message，没有独立 success 字段。

这属于等间隔时间标记和零速度节点约定，**不是** TOTG、TOPP-RA、Ruckig、梯形速度或 jerk 受限规划。下游对带零速度节点的插值可能有中段速度峰值，不能用平均 `Δq/Δt` 替代完整速度/加速度验证。

### `handle_arm` 的启动插值与重放

这是另一个包，`loop.cpp:566–632` 从当前 JointState 到 YAML 第一关节位形做直线插值：

```text
N = ceil(max_j |q_goal[j]-q_start[j]| / interpolation_max_joint_step_rad)
q[k] = q_start + (k/N)*(q_goal-q_start), k=0..N
t[k] = k*interpolation_time_interval_sec
velocity[k] = 0
```

此处第一点时间为零，区别于 IK ROS wrapper 的第一点时间 dt。插值完后按原 YAML 时间戳循环发布录制轨迹，不重新生成末端路径。默认值与随包 YAML 覆盖不同：

| 参数 | C++ 默认（`loop.cpp:259–271`） | `config/handle_arm_trajectory_loop.yaml` |
| --- | --- | --- |
| 插值时间间隔 | 0.1 s | 0.1 s |
| 最大关节采样步长 | 0.1 rad | 0.3 rad |
| 轨迹结束额外等待 | 0.1 s | 0 s |
| JointState 超时 | 0.2 s | 0.2 s |

示例 `config/trajectory.yaml` 只有四个录制关节点，时间为 0、0.4、1.2、1.6 s；不是三级兑换任务，也不能从这些数值推导本项目的 Cartesian 分辨率。停止 toggle 只停止内部等待/后续发送，不撤销控制器已接收的轨迹。

### RRT、MoveIt 与动态限制的证据边界

归档的 IK 理论和包、`handle_arm` 包中没有 RRT/OMPL 路径搜索或碰撞场景实现。v3 `moveit.launch.py` 使用 `MoveItConfigsBuilder(...).to_moveit_configs()` 和 move_group，不能据此确定运行时插件、采样器、RRT 变体或重定时器；没有把依赖包的默认行为验证为上游固定算法。因此本项目如增加 RRT-Connect，须标为新实现/扩展，而非复刻上游 RRT。

v3 `joint_limits.yaml` 对七臂关节声明 1.5 rad/s、1.5 rad/s²，scale=1.0；这是 MoveIt 配置，不是 IK 求解器采用的限制，也不是实测硬件规格。本地原始 URDF 中速度零占位与现有 mock 的 1 rad/s 也须分别说明。公平评测应固定一组明确的模拟运动约束，不能让不同方法各自挑选。

## 4. 三级兑换任务：本地扩展契约

任务根据本地 [rules.md](rules.md) 的官方手册核对结果和现有核心机构建立；不是上游 `axis_free_rotate_ik` 的现成示例。规划阶段采用：

| phase | 路径/目标 | 必须保持的区别 |
| --- | --- | --- |
| `approach` | 当前 ready 位形到 preinsert，再对准柱入口；preinsert 开口距 E 额外 0.04 m 为本地配置 | ready 从 development 选定，不含原显示 home 到 ready；自由运动必须碰撞验证 |
| `insert` | 开口沿柱轴插入 0.1 m | 末端工具位置需通过固定抓取变换换算，不能把 ee 直接当孔口 |
| `lift` | 机构升降 0.1 m | 使用现有机构局部升降方向；非随意世界 Z 平移 |
| `flip` | P 上翻 π/2 | 跟随真实建模 P 枢轴产生的圆弧与方向变化 |
| `twist` | Q 转到任务目标角 | Q 是独立机构轴，使竖起单元侧倾；不是绕单元长轴自转 |
| `dwell` | 在 Q 正确位置持续保持并确认 | 规则 0.3–0.5 s 到位提示窗口；不把执行驻留时间当规划 CPU 耗时 |

基座到装配区、工具到被抓单元的变换应固定并记录；a0、ad、P/Q 旋转的坐标系应一致。释放工具轴自转只有在单元与抓取/碰撞几何允许该自由度时才成立；对刚性非轴对称接触不能仅用自由轴终点误差代替整条携带物几何检查。

## 5. 复刻基线与评测要求

复刻保留本节所列任务公式、固定参数、反向轴处理、迭代上限、warm start、限位以及失败语义。本地 MuJoCo 后端替代 Pinocchio 属于明确的适配；至少对同一 URDF、同一关节顺序、同一 base/ee 的 FK 和末端点 Jacobian 做数值交叉核对。不能只验证 FK 后声称 Jacobian 已一致。

在共同任务包装层增加阶段采样、碰撞验证和满足统一速度/加速度约束的时间戳属于扩展。若增加迭代步长裁剪、相邻步长门槛、最大重试或统一采样间隔，必须记录其数值及来源，并对所有方法使用一致契约。基线不得为了让加速更显著而人为等待、使用无必要超密采样或额外重试。

规划计时从场景/目标已知的请求开始，到完整带时间戳且独立验证通过的轨迹返回结束；在线碰撞与轨迹检查均计入。模型初始化、离线缓存/训练、真实执行与驻留分别记录。需要完整保存失败/超时，比较成功率和耗时分布，而非只汇总成功样本平均值。

上游测试是有用的最小回归来源：`test_axis_free_rotate_ik.cpp` 覆盖 XML/文件 URDF、非法帧和请求、邻近初值固定/自由轴 IK、分离容差、轨迹 warm start、连续性、首失败和逐点覆盖。额外本地验证应针对相同位置但方向反向、绕工具轴自转不改变任务、不可达/限位目标、碰撞连线、阶段顺序和驻留、时间参数运动峰值；仅末端点成功不能证明完整路径可执行。

本文件的源码审计与哈希验证不计作上游构建通过或完整规划基准通过。实际实验结果、适配实现和几何代理精度应在独立结果报告中给出。
