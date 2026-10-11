# Q 轴可转范围：40 位姿在线证据

完整说明见 [Q 范围报告](../../pose40_q_range.md)。正式结果为 35/40 可执行，其中 20 个完整 0° → +90° → −90°，15 个部分转动，5 个未执行。全部之前的 20 个严格往返成功位姿都保留。

本目录根部是正式实现（精确状态缓存 + pre-Q 碰撞预筛）；`controls/no_state_cache/` 是先行逐段验证实现，`controls/no_pre_q_screen/` 是已加精确状态缓存但没有 pre-Q 预筛的实现。三个实验的原始输入均为同一组 40 位姿。每个目录中的 `source/` 与 `execution.json` 记录对应的实际源码和构建哈希，不依赖把当前工作区误当作历史实现。`execution.json` 的 `git_head` 是运行时父版本，实际运行中的未提交改动以源码快照和 SHA-256 为准。

- `requested_poses.csv`、`experiment.json`：原始位姿、共同实际关节起点、安装位置、4 倍速度、采样密度和在线计时边界。
- `runs.csv`、`case_summary.csv`、`scenarios.csv`：全部 40 个请求。`planning_seconds` 包含所有重试及在线验证；`dense_recheck_seconds` 是计时外的无缓存等价审计，不属于执行时间。
- `q_positive_degrees`、`q_final_degrees`、`full_q_sweep`、`q_positive_stop`、`q_negative_stop`：实际角度与停止原因。部分 Q 的规则确认使用实际停止角；`full_q_sweep` 才代表原完整 ±90° 往返。
- `online_attempts.csv`、`trials/`：正式 388 次 waypoint IK、310 次规划调用，含失败种子、候选和轨迹。全部子调用计时计入同一请求。
- `audit.json`、`trials/audit.json`：从源 URDF 重建 FK 的独立审计，正式 40 个最终结果与 310 次规划尝试均通过。失败前缀仅检查有限值与关节范围，不认证为可执行。
- `summary.json`、`comparison.csv`：全体统计及与上一轮严格在线模式的逐位姿对照。
- `trajectories.tar.xz`、`trajectory_sha256.json`、`archive_check.json`：三个实验全部 **964 个逻辑轨迹**，其中 471 份唯一内容，归档内部硬链接去重；逐份读回后 SHA-256 一致。正式实验对应 40 + 310 条，两个对照对应 40 + 256 和 40 + 278 条。
- `ctest_full.log`：21/21 测试通过，包括严格模式隔离、碰撞前截断、反向顺序、驻留连续性、pre-Q 不可放宽、实际起点保留、缓存非量化与失效、无缓存重验。
- 两个视频的 `.json` / `.render.log`、`video_review.json`：输入轨迹和视频哈希、分辨率、帧率、完整解码、5 页终态和部分 Q 转向抽帧记录。视频二进制保留在 `artifacts/level3_planning/videos/`。
- `resources_planning.json`、`resources_rendering.json`：完整 GPU UUID/进程表。规划使用 CPU 0，渲染明确使用 OSMesa llvmpipe、CUDA 隐藏，ffmpeg 两线程；其他任务的 GPU 进程未改动。
- `manifest.json`：本证据目录逐文件校验，不含自身。

恢复全部轨迹到一个不存在的新目录：

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib, json, shutil, tarfile
source = Path('docs/evidence/pose40_q_range')
target = Path('artifacts/level3_planning/q_range_restored')
for name, digest in json.loads((source/'manifest.json').read_text())['files'].items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest() == digest, name
shutil.copytree(source, target)
with tarfile.open(target/'trajectories.tar.xz', 'r:xz') as archive:
    archive.extractall(target, filter='data')
for name, digest in json.loads((target/'trajectory_sha256.json').read_text()).items():
    assert hashlib.sha256((target/name).read_bytes()).hexdigest() == digest, name
print('Restored and verified all 964 trajectories')
PY

CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=1 \
python3 scripts/audit_planning_trajectories.py \
  artifacts/level3_planning/q_range_restored --expected-runs 40 \
  --online-joint-step .004 --online-min-substeps 4 \
  --velocity-limit 4 --acceleration-limit 32
python3 scripts/summarize_online_pose_sweep.py \
  artifacts/level3_planning/q_range_restored --previous docs/evidence/pose40_online
```

原始三个实验目录、旧严格 Q 结果及旧视频均保留。一次性编译/渲染临时目录由 `TemporaryDirectory` 清理，日志限制为每份 1 MiB、保留两份轮转备份。缓存不跨场景配置或请求保留，结束后释放进程资源。
