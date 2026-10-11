# 40 位姿 Q 往返试验的原始证据

算法、参数和逐位姿结果见 [报告](../../pose40_q_sweep.md)。本目录包含 40 个最终结果、533 次完整规划尝试、800 次 ready 尝试及独立审计。19 个失败 case 保留全部有限搜索记录，不作为成功路径播放。

- `requested_poses.csv`、`experiment.json`：开始搜索前固定的 40 位姿和参数。
- `scenarios.csv`、`runs.csv`、`case_summary.csv`：每个位姿最终选定的结果，以及搜索总耗时。
- `ready_search.csv`：全部 800 个实际随机种子关节值、ready IK 输出和起点检查。
- `trials/scenarios.csv`、`trials/runs.csv`：全部 533 次完整调用。
- `audit.json`、`trials/audit.json`：完整成功轨迹的独立 FK/规则/时间审计，以及失败前缀的记录一致性检查。没有第二套独立网格碰撞实现。
- `trajectories.tar.xz`：40 个最终逻辑轨迹加 533 个 trial 逻辑轨迹，共 573 个文件；532 份不同内容以内部硬链接去重。包括空或不完整的失败前缀。
- `trajectory_sha256.json`：每个逻辑轨迹的 SHA-256。归档写完后逐个读取并校验，未因去重省略失败记录。
- `ctest_full.log`、`ctest_contract.log`：20 项完整测试，以及随后补充完整 Q 往返成功用例后的单项回归。
- `manifest.json`：源文件、参数、证据、视频的校验和与验证摘要。

原始未压缩数据保留在仓库的 `artifacts/level3_planning/pose40_q_sweep/`。要从本目录恢复并重新审计，可在仓库根目录执行以下命令，目标路径须不存在：

```bash
python - <<'PY'
import hashlib, json, shutil, tarfile
from pathlib import Path
source = Path('docs/evidence/pose40_q_sweep')
target = Path('artifacts/level3_planning/pose40_restored')
manifest = json.loads((source/'manifest.json').read_text())
for name, digest in manifest['files'].items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest() == digest, name
shutil.copytree(source, target)
with tarfile.open(target/'trajectories.tar.xz', 'r:xz') as archive:
    archive.extractall(target, filter='data')
for name, digest in json.loads((target/'trajectory_sha256.json').read_text()).items():
    assert hashlib.sha256((target/name).read_bytes()).hexdigest() == digest, name
print('Verified and restored all 573 logical trajectories')
PY

OPENBLAS_NUM_THREADS=1 python scripts/audit_planning_trajectories.py \
  artifacts/level3_planning/pose40_restored --expected-runs 40 \
  --velocity-limit 4 --acceleration-limit 32
OPENBLAS_NUM_THREADS=1 python scripts/audit_planning_trajectories.py \
  artifacts/level3_planning/pose40_restored/trials --expected-runs 533 \
  --velocity-limit 4 --acceleration-limit 32

python scripts/render_pose_sweep.py artifacts/level3_planning/pose40_restored \
  --output artifacts/level3_planning/videos/pose40-restored.mp4
```

视频是固定夹持、按已验证时间戳进行的运动学回放。汇总视频显示全部 40 个独立场景，失败项始终静止；成功项按 `0° → +90° → −90°` 顺序完成 Q 往返。碰撞验证是离散采样，未作真实电机、负载或抓取动力学验证。
