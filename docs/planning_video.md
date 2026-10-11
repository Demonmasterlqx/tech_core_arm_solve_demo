# 三级兑换视频：更近的安装位置、降低基座与 4 倍运动速度

后续的 **40 位姿、Q 从 0° 到 +90° 再到 −90°** 试验、视频及算法参数见 [40 位姿报告](pose40_q_sweep.md)。本页保留先前单场景视频的记录。

## 单场景视频：基座降低 100 mm，pitch −10°，roll +10°

按用户要求，在先前更近的安装位置上，将机器人根刚体和固定安装座沿世界 Z **实际下移 0.10 m**，核心与地面不随之移动。`Scenario::arm_base_z=-0.10`，安装平移仍为 `(0.20, 0.20, -0.25)` m。核心初始姿态为 `phi=80°`、`alpha=10°`，即本模型约定的 **pitch = phi − 90° = −10°、roll = +10°**。规则对 `phi` 的范围限制是 `[0°,90°]`。更改的是仿真中的空间位置和姿态。

此前视频约 47 秒（更近的版本为 50.296 秒），主要来自逐路径点零速度的三次插值、每段至少 0.1 秒及模拟加速度上限 2 rad/s²。现在 `PlanOptions::motion_speed_scale=4` 将每个运动区间的时间除以 4；路径点和碰撞采样密度保留，三级规则的最终 0.5 秒驻留保留。视频仍以 **1 倍播放速度**展示重新计时后的轨迹。

| 当前视频 | 数值 |
| --- | ---: |
| 单次成功规划，给定已选定的 ready | 0.275902 s |
| 同一条新路径按原参数化速度的时长 | 48.320600 s |
| 4 倍运动速度后的轨迹时长，含驻留 | 12.455150 s |
| 最终驻留 | 0.500000 s |
| 路径点数 | 258 |
| 额外密采样碰撞复查，缓存关闭 | 1612 点，通过 |
| 独立 URDF FK / 规则 / 时间审计 | 1665 点，通过 |
| 最大独立位置误差 | 0.045148 mm |
| 最大独立轴角误差 | 0.000231670 rad |
| 峰值关节速度 | 0.721055 rad/s |
| 峰值关节加速度 | 32.000000 rad/s² |
| Q 连续对准时长 | 0.737740 s |

总时长满足 `(48.320600 − 0.5) / 4 + 0.5 = 12.455150 s`。时间压缩会使速度扩大 4 倍、加速度扩大 16 倍，模拟验证限值显式变为 4 rad/s 和 32 rad/s²；默认 1 倍配置保持原值。这里沿用运动学回放和固定刚性携带假设，没有验证电机力矩、负载或真实硬件的动态可执行性。碰撞通过离散加密采样检查；独立 Python 审计检查 FK、规则和时间，没有第二套独立网格碰撞实现。

原 ready 分支在新姿态下出现 j2/j4 自碰。固定种子 `20261012` 的离线 ready IK 搜索共尝试 12 次：前 11 次被起点碰撞门禁拒绝，第 12 个候选 `lowered_tilted_4x_s11` 通过起点检查及完整规划。0.275902 s 包含该次规划的在线碰撞验证，不包括离线 ready 搜索、额外密采样复查或视频渲染。所有尝试、最终轨迹和审计均保存在 [本次证据目录](evidence/level3_video_lowered_tilted_4x/manifest.json)。这是单场景演示，不并入原先冻结的 621 次规划基准，不构成覆盖率提升结论。

本次构建及 **20/20 CTest** 通过，包括同路径各运动段缩为四分之一、驻留保持 0.5 秒、速度和加速度分别变为 4/16 倍，以及机器人下移时核心和地面保持原位的检查。旧基准仍对应 `afc7f852771289c2070f642eda5e51613e1212a0`；当前增加了位置和时间缩放能力，源文件哈希已改变。

成品为 `artifacts/level3_planning/videos/level3-lowered-tilted-motion4x.mp4`。按归档轨迹重建：

```bash
python scripts/render_planning_video.py docs/evidence/level3_video_lowered_tilted_4x \
  --case lowered_tilted_4x_s11 \
  --output artifacts/level3_planning/videos/level3-lowered-tilted-motion4x-replay.mp4

OPENBLAS_NUM_THREADS=1 python scripts/audit_planning_trajectories.py \
  docs/evidence/level3_video_lowered_tilted_4x --expected-runs 1 \
  --velocity-limit 4 --acceleration-limit 32
```

`--motion-speed-scale 4` 是规划器及基准脚本的运动时间缩放选项，`--speed` 是视频脚本的播放速度选项；本次视频使用默认的 `--speed 1`。场景源文件 `plan_scene.cpp` 也已归档，使用 `PlanningScene`、确定性 ready 搜索和 `RigidCache` 重新求解。

## 前一版视频：实际水平间距缩短 100 mm

按用户要求，机械臂基座到核心初始 E 点的**水平距离从 390 mm 缩短到 290 mm**。原机械臂基座坐标系保持不动，将 `Scenario::assembly_translation.x` 从 0.30 m 改为 0.20 m；所有物体尺度、夹爪—单元变换、地面高度、机构目标和碰撞阈值保持一致。这是实际相对位置变化。

直接沿用原 ready 分支会发生 j2/j4 自碰。离线使用固定随机种子 `20261011` 搜索更适合新安装位置的 ready 姿态，在候选 26 得到完整成功路径 `closer_x200_s26`。保留了 7 个直接平移探针、27 个起始姿态尝试，以及其中通过起点检查后的 12 次完整规划尝试记录。新演示不并入原先的 621 次冻结基准，也不代表覆盖范围或成功率得到提高。

| 前一版视频 | 数值 |
| --- | ---: |
| 单次成功规划，给定已选定的 ready | 0.338193 s |
| 参数化轨迹时长 | 50.296019 s |
| 额外密采样碰撞复查 | 1526 点，通过，缓存关闭 |
| 独立 URDF FK 审计 | 1597 点，通过 |
| 最大独立位置误差 | 0.052216 mm |
| 最大独立轴角误差 | 0.000146578 rad |
| Q 连续对准时长 | 1.651790 s |

0.338193 s 包含本次规划的在线碰撞验证，**不包括上述离线起始姿态搜索**。视频按三次曲线 `3u²−2u³` 插值关节和机构状态，与规划验证一致；调用原 C++ `PlanningScene::set_state`，不以重新积分动力学代替已验证轨迹。每次渲染前再运行密采样 validator。它展示固定刚性携带下的三级轨迹回放，不是抓取或真实控制执行证明。

完整输入、成功和失败记录、生成探针与审计在 [证据目录](evidence/level3_video_closer/manifest.json)。该目录中的 `runs.csv` 保留全部 12 次规划，15 个未通过 ready IK/起点门禁的尝试另列于 `placement_scan.csv`。

## 重建视频

先按 [规划报告](planning_results.md) 构建 `build-planning`。视频额外需要 `libosmesa6-dev` 与 `ffmpeg`；不需要 Python MuJoCo 包。以下命令使用已归档的真实轨迹，输出应使用新路径：

```bash
python scripts/render_planning_video.py docs/evidence/level3_video_closer \
  --case closer_x200_s26 \
  --output artifacts/level3_planning/videos/level3-closer-10cm-replay.mp4
```

默认 1600×900、24 fps、1 倍速，包含全臂和夹爪/核心特写，最后保留 2 秒终态。以 `--still 25 --output artifacts/level3_planning/videos/closer-25s.png` 可输出指定轨迹时间的一帧；`--speed 4` 可生成 4 倍速版本。

脚本强制 `CUDA_VISIBLE_DEVICES=''`、Mesa llvmpipe 和 4 个 CPU 渲染线程，并核验实际 OpenGL renderer；ffmpeg 编码使用 2 个 CPU 线程。RGB 直接通过管道交给编码器，不落盘保存帧序列。临时编译文件在结束或异常时清理；日志 1 MiB 轮转、最多两份备份。最终视频先写临时文件，经 ffprobe 验证完整帧数后原子改名，同时输出 SHA-256、实际 renderer、模型/轨迹哈希和时长 JSON。

前一版成品保留于 `artifacts/level3_planning/videos/level3-closer-10cm.mp4`。原始 390 mm 名义视频也保留供比较；该轮交付为本页开头的降低基座、倾斜核心和 4 倍运动速度版本。
