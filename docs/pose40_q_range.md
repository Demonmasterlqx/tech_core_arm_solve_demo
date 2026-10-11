# 40 位姿在线规划：Q 轴尽量转动

**35/40 个位姿得到可执行轨迹：20 个完整完成 0° → +90° → −90°，15 个在验证通过的角度处转向或停止；另外 5 个未执行。** 之前在线严格往返成功的 20 个位姿全部保留。可执行方案的完整在线规划耗时为 **0.878932–2.940423 s**，中位数 **1.915597 s**；这里不包含轨迹执行时间。

本次只放宽最后 Q 阶段的到角要求。40 个输入位姿、实际起点、实际安装距离、机械臂基座 Z = −100 mm、核心 pitch/roll、4 倍运动速度均与[上一轮在线试验](pose40_online.md)一致。插入 100 mm、抬升 100 mm 和 P 翻转 90° 仍必须完整通过。Q 保持在机构原有的 **−90°～+90°** 行程内。

## 可执行范围的含义

新增 `Scenario::q_best_effort`，默认关闭，原严格 Q sweep 仍可复现。新模式首先尝试正转，遇到 IK、关节限制、碰撞/间隙或预算限制时停在最后完整验证通过的 1° 步长端点；沿正向关节路径反向返回 Q=0，再尽量向 −90° 求解。每一步采用相同的 cubic 插值、密碰撞与跟踪检查，失败段不会进入返回轨迹。最后驻留 0.5 s，重新检查整条路径。

多次在线候选中，保留已验证的最大总行程 `2*Q_peak − Q_final`；完整 270° 往返一旦找到即可返回。后续失败不会覆盖此前的可执行候选。若可选扩展整段复查失败，只有之前已完成全路径检查的零 Q 驻留候选可以回退；正式试验没有出现零 Q 运动的接受结果。前面的插入、抬升、P 翻转失败时不会作为“部分 Q 成功”返回。

这里的范围是**在线预算与实际搜索到的 IK 分支内找到的可执行范围**，不是全局最大可达范围证明。`pose12` 和 `pose23` 的角度受在线预算影响，不能把其终点当作机械极限。原严格往返成功率仍为 20/40；35/40 指用户新要求下的可执行率，部分 Q 不等同于原来的完整 ±90° 任务。新模式 `rule_success` 按实际停止角确认，`full_q_sweep` 单独记录原端点是否都到达。

## 在线计算和碰撞检查

复用任务雅可比 predictor + Reduced5D DLS，不读取旧的逐位姿 ready 解或轨迹，不使用 atlas，也不在请求间 warm start。每个请求仍从同一个实际关节输入开始，在线选出的 waypoint 包含连续的实际起点连接。原始输入 CSV 的 SHA-256 为 `4341706108d9ed926af719702e0061b8d85e3cbca320dd91898a8d3bff5bed08`。

重试共用 3 s 总时限，最多 96 次 waypoint IK、24 次规划调用。搜索在 2.94 s 停止，给返回留 60 ms 余量；可选 Q 扩展另按本请求已测得的验证成本预留整条路径复查时间。接受结果仍要求实际测得的完整在线耗时小于 3 s。它是软件截止检查，并非硬实时保证。时间包括缓存清空、实际起点检查、全部失败重试、waypoint IK、连续连接、任务 IK、时间参数化、预筛和完整密验证；不包括一次性模型加载、CSV I/O、计时后的无缓存等价审计、渲染和执行。

本次新增两项节省重复工作的处理：

- 精确状态碰撞缓存：键包含全部 7 个关节、插入/抬升/P/Q 的 4 个机构量和 clearance，共 12 个原始 double，不量化。`configure()` 清空并禁用缓存，没有跨请求复用；命中仍恢复场景状态。全部请求累计 160,108 次命中。
- pre-Q 端点碰撞预筛：提前拒绝已经在插入/抬升/P 翻转端点发生碰撞的候选，减少对失败候选的密检查成本。预筛只能拒绝，不能用于接受。

正式接受的采样数依旧是每段 `max(4, ceil(1.5*max_abs_dq/0.004))`。自由空间间隙为 1 mm，指定装配/抓持接触容限为 0.1 mm，没有降低几何检查要求。在线完整检查后，关闭刚体缓存和精确状态缓存，以相同采样密度复查全部接受轨迹；不一致会终止试验。这项计时外审计耗时单列，不能把它误当作在线规划或执行时间。

## 结果和验证

| 项目 | 正式结果 |
| --- | ---: |
| 可执行 / 完整 Q / 部分 Q / 未执行 | 35 / 20 / 15 / 5 |
| 可执行在线耗时 min / median / P95 / max | 0.878932 / 1.915597 / 2.940367 / 2.940423 s |
| 全部 40 请求在线耗时合计 | 76.760258 s |
| 在线 waypoint IK / 规划调用 | 388 / 310 |
| 接受路径执行时间范围（4 倍运动、1 倍播放） | 17.612–26.180 s |
| 全路径密碰撞样本 | 101,732，缓存开/关结果一致 |
| 独立 URDF FK 样本 | 203,429 |
| 独立 FK 最大位置 / 轴角误差 | 0.210967 mm / 0.001728403 rad |
| 完整 CTest | 21/21 通过 |
| 40 最终结果 / 310 次规划尝试独立审计 | 全部通过，0 错误 |

`pose06`、`pose15`、`pose30` 达到 24 次规划调用上限，最后一次在 P 翻转 IK 失败；`pose16`、`pose20` 在时限内没有得到完整验证通过的 pre-Q 轨迹。这些是本次搜索失败，不是不可达证明。所有失败都保留在结果和视频中，不执行未验证前缀。

独立审计从固定版本源 URDF 重建 FK，使用两倍在线分段采样，并检查实际起点、机构顺序、实际 Q 端点、至少 0.4 s 到角驻留和解析速度/加速度上界。它独立于规划器 FK，但不是第二套网格碰撞引擎。全部记录仍是固定夹持的运动学模拟和离散碰撞验证，不代表硬件驱动、负载或连续时间无碰撞证明。

本轮每个位姿单次正式记录，CPU 0 单线程在线规划。为定位开销做过的两个先行实现也保留全体 40 位姿和失败记录；下表是共享机器上的观测，不作为重复试验置信区间。

| 实现 | 可执行 | 完整 Q | 部分 Q | 零 Q 运动 | 可执行在线中位数 / s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 逐段验证 | 30 | 11 | 18 | 1 | 2.940 |
| 增加精确状态缓存 | 32 | 16 | 16 | 0 | 2.515 |
| 缓存 + pre-Q 碰撞预筛（正式） | 35 | 20 | 15 | 0 | 1.916 |

## 15 个部分转动位姿

| 位姿 | 实际 Q 序列 | 在线耗时 / s | 限制来源 |
| --- | --- | ---: | --- |
| pose04 | 0° → +72° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose08 | 0° → +88° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose11 | 0° → +64° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose12 | 0° → +43° → 0° | 2.940 | 在线搜索/验证预算 |
| pose13 | 0° → +68° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose14 | 0° → +90° → -29° | 2.940 | 反向碰撞/间隙边界 |
| pose18 | 0° → +73° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose19 | 0° → +6° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose21 | 0° → +78° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose23 | 0° → +90° → -1° | 2.658 | 在线搜索/验证预算 |
| pose25 | 0° → +60° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose26 | 0° → +60° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose31 | 0° → +67° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose32 | 0° → +72° → -90° | 2.940 | 正向碰撞/间隙边界 |
| pose40 | 0° → +76° → -90° | 2.940 | 正向碰撞/间隙边界 |

`pose11` 特写为 **0° → +64° → −90°**：正向的下一个区间触发腕部 `j4`/`j6` 几何间隙检查（0.992 mm < 要求 1 mm），该区间被丢弃，从已验证的 +64° 转向。其他 40 位姿的输入、结果、范围和旧结果对照见[逐位姿表](evidence/pose40_q_range/comparison.csv)。

## 视频

汇总视频展示全部 40 个输入，分 5 页，每页 8 个独立场景；青色为完整 Q，琥珀色为部分 Q，红色为未执行。每格标注实际正向终点、最终 Q 角和完整在线耗时，部分转动使用 `SAFE STOP`，不会标为完整兑换。特写同时展示整臂与夹爪/核心。

- [40 位姿汇总](../artifacts/level3_planning/videos/level3-40poses-online-q-range.mp4)：1920×1080，20 fps，2758 帧，137.90 s。
- [pose11 部分 Q 特写](../artifacts/level3_planning/videos/level3-online-q-range-pose11-detail.mp4)：1600×900，24 fps，594 帧，24.75 s。

视频按验证轨迹的时间戳以 1 倍播放；运动速度本身保持 4 倍。使用 CPU OSMesa llvmpipe，CUDA 隐藏，ffmpeg 两线程。视频元数据、完整解码与抽帧检查及资源退出记录见[证据索引](evidence/pose40_q_range/README.md)。

## 复现

```bash
cmake -S . -B build-q-best-effort -DCMAKE_BUILD_TYPE=Release -DTECH_CORE_VIEWER=OFF
cmake --build build-q-best-effort -j4
CUDA_VISIBLE_DEVICES='' ctest --test-dir build-q-best-effort --output-on-failure

CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=1 \
python3 scripts/run_pose_sweep.py --online --retry --best-effort-q \
  --build-dir build-q-best-effort \
  --poses docs/evidence/pose40_q_range/requested_poses.csv \
  --output artifacts/level3_planning/q_range_reproduction

CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=1 \
python3 scripts/audit_planning_trajectories.py \
  artifacts/level3_planning/q_range_reproduction --expected-runs 40 \
  --online-joint-step .004 --online-min-substeps 4 \
  --velocity-limit 4 --acceleration-limit 32
python3 scripts/summarize_online_pose_sweep.py \
  artifacts/level3_planning/q_range_reproduction \
  --previous docs/evidence/pose40_online

python3 scripts/render_pose_sweep.py artifacts/level3_planning/q_range_reproduction \
  --build-dir build-q-best-effort --output artifacts/level3_planning/videos/q-range-reproduction.mp4
python3 scripts/render_planning_video.py artifacts/level3_planning/q_range_reproduction \
  --build-dir build-q-best-effort --case pose11 \
  --output artifacts/level3_planning/videos/q-range-pose11-reproduction.mp4
```

输出目录必须不存在；原有严格 Q 试验和三个新实验都已保留。正式原始文件位于 `artifacts/level3_planning/pose40_online_q_range_screened/`。省略 `--best-effort-q` 即恢复原严格 ±90° 验收逻辑。限时搜索在机器负载变化时可能产生不同的已验证角度，必须重新审计实际结果。
