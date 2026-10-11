# 40 位姿在线重跑证据

完整说明见 [在线重跑报告](../../pose40_online.md)。本目录保留正式在线重试的 40 个最终结果、354 次规划调用、451 次 waypoint IK，以及先行无重试对照的全部 40 个结果。20 个正式成功请求的总在线规划时间均小于 3 s；16 个超时和 4 个尝试次数用尽的请求均不执行。

- `requested_poses.csv`、`experiment.json`：与上一轮逐字节相同的核心输入、统一实际起点、在线算法和计时边界。
- `runs.csv`、`scenarios.csv`、`case_summary.csv`：全部 40 个请求。`planning_seconds` 包含整次请求的失败重试；`dense_recheck_seconds` 是计时外的无缓存一致性审计。
- `online_attempts.csv`：451 个 waypoint IK 的种子、输出、状态；另含 40 个直接候选。未进入完整规划的失败 IK 和不安全 waypoint 也保留。
- `trials/runs.csv`、`trials/scenarios.csv`：354 次完整规划调用。这里每行 `planning_seconds` 是该子调用耗时，最终 `runs.csv` 则包含同一 case 的全部子调用和起点求解成本。
- `audit.json`、`trials/audit.json`：独立源 URDF FK、机构、关节限制和时序审计；失败前缀不被认证为可执行轨迹。
- `summary.json`、`comparison.csv`：完整人口统计及与上一轮逐 case 对照，不只统计成功项。
- `direct_control/`：先行不重试对照，5/40 成功；`source/` 保存当时的批量驱动，核心库版本见该目录的 `execution.json`。
- `trajectories.tar.xz`、`trajectory_sha256.json`：40 个正式最终结果、354 个正式试验结果及 40 个无重试结果，共 **434 个逻辑轨迹**。354 份唯一内容用归档内部硬链接去重；包括空或不完整的失败前缀。每个归档内容均已读回校验。
- `ctest_full.log`：20/20 完整测试的日志，包括实际起点、waypoint 连接、密缓存/无缓存一致性和超时负例。
- 视频元数据、完整解码检查、抽帧时刻、资源记录及 `manifest.json`：来源、SHA-256、CPU 软件渲染和进程清理证据。

视频原文件保留在 `artifacts/level3_planning/videos/`，元数据及 SHA-256 入库；未把视频二进制加入 Git。原始未压缩实验数据保留在 `artifacts/level3_planning/pose40_online_retry/` 和 `pose40_online/`。

在仓库根目录恢复记录，目标目录必须不存在：

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib, json, shutil, tarfile
source = Path('docs/evidence/pose40_online')
target = Path('artifacts/level3_planning/pose40_online_restored')
manifest = json.loads((source/'manifest.json').read_text())
for name, digest in manifest['files'].items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest() == digest, name
shutil.copytree(source, target)
with tarfile.open(target/'trajectories.tar.xz', 'r:xz') as archive:
    archive.extractall(target, filter='data')
for name, digest in json.loads((target/'trajectory_sha256.json').read_text()).items():
    assert hashlib.sha256((target/name).read_bytes()).hexdigest() == digest, name
print('Restored and verified all 434 logical trajectories')
PY

CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=1 \
python3 scripts/audit_planning_trajectories.py \
  artifacts/level3_planning/pose40_online_restored --expected-runs 40 \
  --online-joint-step .004 --online-min-substeps 4 \
  --velocity-limit 4 --acceleration-limit 32

python3 scripts/summarize_online_pose_sweep.py \
  artifacts/level3_planning/pose40_online_restored \
  --previous docs/evidence/pose40_q_sweep \
  --direct docs/evidence/pose40_online/direct_control
```

所有视频为按已验证时间戳插值的固定夹持运动学回放。视频始终从每个请求的统一实际关节输入开始，红色失败格保持静止；没有将在线求得的 waypoint 当成瞬移后的实际起点。
