# 40 位姿在线规划重跑

同一组 40 个科技核心位姿已改用在线规划重新尝试：**20/40 成功，成功请求的完整在线耗时中位数 1.229866 s，最大 2.808866 s**。另外 16 个请求到达 3 s 截止时间，4 个达到候选次数上限，最后原因为 IK 失败。全部失败保留，没有替换位姿或执行被拒绝的轨迹。

与[上一轮](pose40_q_sweep.md)相比，成功数从 21 降到 20；唯一未保留的成功位姿是 `pose21`，本次超时。新的起点和计时边界更严格，不能把上一轮选中轨迹的 0.535 s 中位数直接作为完整在线请求的对照。

## 输入和实际起点

复用上一轮 `requested_poses.csv` 的原始字节，SHA-256 为 `4341706108d9ed926af719702e0061b8d85e3cbca320dd91898a8d3bff5bed08`。40 个核心位置、yaw/pitch/roll、随机采样和顺序均保持原值。

- 装配坐标系 O 到世界系的平移为 `(0.20, 0.20, -0.25) m`，yaw 为 π；机械臂实际基座 Z 为 −0.10 m。
- 核心 pitch 为 `phi−90°`，范围 −20°～−2°；roll 范围 −15°～+15°。
- 三级动作：接近、插入 100 mm、抬升 100 mm、P 翻转 90°、**Q：0° → +90° → −90°**、驻留 0.5 s、规则确认。
- 运动速度缩放仍为 4，视频播放为 1 倍，规划时间不含执行时间。

所有请求使用同一个模拟实际关节输入，单位 rad：

```text
[-0.58298870078787879, 1.4014363481415861, 1.4956286729240151,
  0.51538441396359136, -1.5264021873645037, -0.58813609807777278,
  2.7857137959190528]
```

这是之前参考场景的已知姿态，**不是针对这 40 个位姿逐个选出的 ready**，也不代表从任意机械臂当前姿态都能获得相同成功率。每个试验独立从这个姿态复位；视频各格是独立场景，不把相邻 case 拼成一条连续执行任务。

## 在线算法和计时

在线指接收到核心位姿和关节输入后计算整段轨迹。算法是 HKUST 任务雅可比对应的 Jacobian predictor + Reduced5D DLS，加请求内刚体碰撞缓存 `RigidCache`，没有 atlas、逐位姿离线解或跨请求轨迹 warm start。原始公式、几何模型和接触例外见[算法说明](pose40_q_sweep.md)和[场景审计](planning_scene_audit.md)。

每个请求开始一次 `steady_clock` 计时，并共用一个 **3 s** 截止时间：

1. 从实际输入关节姿态直接尝试完整规划。
2. 如失败且还有时间，在线解初始 E 点沿轴向外侧 60 mm 的候选 waypoint。首个种子是实际起点；之后围绕该固定种子作确定性高斯扰动。每个 case 的 RNG 为 `20261013 + 零基序号`，不读取上一 case 的成功姿态。
3. 对候选 waypoint 生成从实际起点出发的关节直线连接，最大关节分量间隔 0.02 rad，用 FK 得到各连接点的任务目标；接着规划接近和完整兑换动作。候选 waypoint 不会覆盖实际起点，视频不会瞬移到候选姿态。
4. 对连接和后续动作一起进行时间参数化、密碰撞、跟踪误差及规则验证。只有整条路径在截止时间内完成检查才接受，否则保留失败并重试。

最多 96 次 waypoint IK、24 次完整规划尝试，先达到次数上限或总时限即停止。IK 内部、连接生成和验证循环都检查剩余时间。每次子规划只获得剩余时间，不重新获得 3 s。候选 2–31 的关节高斯扰动标准差为 0.35 rad，32–63 为 0.7 rad，64–96 为 1.2 rad。Waypoint IK 使用 FullStep、600 次上限及位置/轴角各 `1e-7` 容差；完整路径 IK 使用 300 次上限和位置/轴角各 `1e-4` 容差。每次重试的种子、候选、失败原因和完整/不完整轨迹全部留档。

**在线耗时包括**场景配置与缓存清空、实际起点检查、全部 waypoint IK 和失败重试、实际起点连接、完整任务 IK、时间参数化，以及密碰撞/跟踪/规则验证。日志与 CSV 在请求计时结束后写出。

**在线耗时不包括**一次性模型加载（本次 0.952110 s）、文件输入输出、额外的关闭缓存一致性审计、渲染或执行。模型保持加载，但刚体缓存每次场景配置都会清空；首次填充缓存的成本计入请求。该“在线”模式不是执行中的视觉伺服或移动障碍重规划。

## 碰撞与独立验证

本次将在线验证加密为每段 `max(4, ceil(1.5*max_abs_dq/0.004))` 个子步，与上一轮额外密复查相同。自由空间间隙仍为 1 mm，已定义装配和固定夹持接触的数值容限为 0.1 mm；未放宽碰撞阈值。接近连接同样检查机械臂、所携带单元、核心和地面等场景几何。

刚体缓存只复用焊接于同一刚体上的几何对距离，运动物体之间的距离继续求解。成功请求在线即完成全部密检查；随后关闭缓存，以**完全相同的采样、容差和轨迹**做一致性审计，有任何不一致便中止试验。后者本次合计 42.590559 s，单列为实验验证成本，没有计入在线延迟，也没有用它补充遗漏的在线采样。

独立 Python 审计从固定版本的源 URDF 重新计算 FK、机构几何、关节限制、插值速度/加速度以及 Q 顺序。其采样数是本次在线分段采样的两倍；这不是另一套独立网格碰撞实现。失败前缀只检查数据完整性和关节限位，不认证为可执行路径。

| 验证项 | 结果 |
| --- | ---: |
| 完整 CTest | 20/20 通过 |
| 最终 40 条结果独立审计 | 通过，0 错误 |
| 全部 354 次规划尝试独立审计 | 通过，0 错误 |
| 成功路径密碰撞样本数 | 59,977；缓存开/关结果一致 |
| 成功路径独立 FK 样本数 | 119,934 |
| 最大位置误差 | 0.084545 mm |
| 最大轴角误差 | 0.000448181 rad |
| 模拟速度/加速度上限 | 4 rad/s / 32 rad/s² |

所有轨迹仍是固定夹持的运动学模拟，不证明真实驱动力、夹持力、负载能力或连续时间无碰撞。新增回归检查实际起点保留、候选连接显式存在、连接的无缓存验证，以及过期请求不能被接受。

## 实测结果

先做了不重试的在线对照，成功 5/40，成功耗时 0.914–1.172 s。加入上述同一时限内的在线重试后，新增 15 个成功，达到 20/40。两批均保留原始记录；正式视频使用带重试的结果。

| 指标 | 在线重试结果 |
| --- | ---: |
| 成功 / 超时 / 尝试次数用尽 | 20 / 16 / 4 |
| 实际 waypoint IK / 完整规划尝试 | 451 / 354 |
| 成功在线耗时最小 / 中位 / P95 / 最大 | 0.707575 / 1.229866 / 2.732097 / 2.808866 s |
| 所有 40 个请求在线耗时合计 | 76.660044 s |
| 最慢失败请求返回耗时 | 3.000282 s |
| 成功轨迹执行时长范围 | 23.781–26.187 s |
| 包含模型加载、无缓存审计及保存的进程 wall time | 120.780703 s |

354 次规划调用中，20 次完整通过、205 次因碰撞/间隙拒绝、113 次因 IK 拒绝、16 次在检查过程中到达截止时间。碰撞失败多发生在腕部非相邻几何之间；超时不是不可达证明。4 个提前达到 24 次规划上限的 case 是 `pose06/15/19/30`。

截止检查是软件轮询，失败请求最多约晚 0.282 ms 返回，**不声称硬实时保证或 40/40 都成功**。成功返回均严格小于 3 s。这里是共享机器、固定逻辑 CPU 0、单线程 C++ 规划下的一次观测，不是重复运行置信区间。上一轮起点不同、计时还排除搜索和失败，因此不计算两轮间的算法加速百分比。

逐位姿结果和上一轮对照见 [comparison.csv](evidence/pose40_online/comparison.csv)，成功/失败明细、每次尝试与计时边界见 [证据索引](evidence/pose40_online/README.md)。

## 视频和复现

汇总视频展示全部 40 个输入，分 5 页、每页 8 格；成功项显示实际执行，失败项静态标红。每格标明完整在线延迟。`pose02` 特写展示在线选择 waypoint 后从共同实际起点连接、再完成 Q 往返的全过程。

- 汇总：`artifacts/level3_planning/videos/level3-40poses-online.mp4`，1920×1080、20 fps、2,763 帧、138.15 s、1 倍播放。
- 特写：`artifacts/level3_planning/videos/level3-online-pose02-detail.mp4`，1600×900、24 fps、677 帧、约 28.21 s、1 倍播放。
- 原始正式结果：`artifacts/level3_planning/pose40_online_retry/`。
- 无重试对照：`artifacts/level3_planning/pose40_online/`。
- 入库证据：`docs/evidence/pose40_online/`。

两个视频均已完整解码验证，并核对汇总五页的终态、特写起点及 Q 正负 90° 画面。434 个逻辑轨迹（含无重试对照）已压缩归档，逐个读回并完整解压校验 SHA-256；临时验证目录随后自动清理。

规划和渲染均不用 GPU；渲染使用 OSMesa `llvmpipe`，每进程 4 个渲染线程，ffmpeg 2 个编码线程。CUDA 隐藏，完整 GPU 进程表与物理 UUID 在规划、渲染、结束时记录。日志按 1 MiB 轮转，最多保留两份备份；临时编译、编码、解压目录自动清理，旧实验和本次全部结果保留。

在仓库根目录运行，输出目录/文件必须尚不存在：

```bash
cmake --build build-planning -j4
CUDA_VISIBLE_DEVICES='' ctest --test-dir build-planning --output-on-failure

CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 scripts/run_pose_sweep.py --online --retry \
  --poses docs/evidence/pose40_q_sweep/requested_poses.csv \
  --output artifacts/level3_planning/pose40_online_reproduced

CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=1 \
python3 scripts/audit_planning_trajectories.py \
  artifacts/level3_planning/pose40_online_reproduced --expected-runs 40 \
  --online-joint-step .004 --online-min-substeps 4 \
  --velocity-limit 4 --acceleration-limit 32

python3 scripts/summarize_online_pose_sweep.py \
  artifacts/level3_planning/pose40_online_reproduced \
  --previous docs/evidence/pose40_q_sweep

python3 scripts/render_pose_sweep.py artifacts/level3_planning/pose40_online_reproduced \
  --output artifacts/level3_planning/videos/level3-40poses-online-reproduced.mp4
python3 scripts/render_planning_video.py artifacts/level3_planning/pose40_online_reproduced \
  --case pose02 --output artifacts/level3_planning/videos/level3-online-detail-reproduced.mp4
```

去掉 `--retry` 即为同一起点、不尝试额外 waypoint 的在线对照。严格 wall-clock 截止会使共享机器上的尝试次数及接近时限的结果随负载变化；渲染特写时需要选择当前重跑实际成功的 case。
