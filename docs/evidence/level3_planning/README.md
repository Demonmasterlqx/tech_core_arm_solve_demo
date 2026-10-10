# 冻结三级规划证据

结果解读见 [planning_results.md](../../planning_results.md)。本目录归档本地实现提交 `afc7f852771289c2070f642eda5e51613e1212a0` 的 **9 方法 × 23 场景 × 3 重复 = 621 条**请求；原输出仍保留在 `artifacts/level3_planning/frozen_621/`，不要覆盖。

| 文件 | 内容 |
| --- | --- |
| `runs.csv` | 621 条运行记录，包含失败、碰撞/IK 计数、在线及 dense 复查计时 |
| `scenarios.csv` | 全部场景位姿、目标、固定起点和分组 |
| `timing.csv` / `atlas_build.csv` | 模型冷加载、Atlas 构建及开发模板来源 |
| `summary.json` / `summary.md` | 成功、失败、分组时间分位数及同请求配对统计 |
| `environment.json` / `benchmark.log` | 完整命令、CPU/线程环境、进程终态和冻结的 45 个源码哈希 |
| `audit.json` | 原 URDF 独立 FK、机构、抓持、运动峰值及全量记录审计，valid=true |
| `ctest.log` | 最终 20 个 CTest 测试通过日志 |
| `mating_compare.txt` / `planning_scene_test.txt` / `reproducibility.json` | 几何配合、场景和生成器可复现检查 |
| `manifest.json` | 每条轨迹的逻辑路径、大小、SHA-256；归档哈希；154 个加载网格的冻结哈希 |
| `trajectories.tar.gz` | 621 个逻辑 CSV 路径，包含失败前缀；相同内容用归档内部硬链接去重 |

轨迹归档有 168 份唯一内容，5,213,124 字节。硬链接目标均位于归档内部，按完整归档解压会恢复全部 621 个文件名。SHA-256：

```text
42f7eea5a4c517cbd02121c6de561d8e8fb66b048773a0b245e125b00f7b736d
```

在仓库根目录可不解压地检查全部轨迹；这也验证硬链接解析后的每一份逻辑内容：

```bash
python - <<'PY'
import hashlib, json, tarfile
from pathlib import Path
p = Path('docs/evidence/level3_planning')
m = json.loads((p / 'manifest.json').read_text())
archive = p / 'trajectories.tar.gz'
assert archive.stat().st_size == m['archive_bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest() == m['archive_sha256']
with tarfile.open(archive, 'r:gz') as tar:
    expected = {r['path'] for r in m['trajectories']}
    members = tar.getmembers()
    assert len(members) == len(expected) == 621
    assert {f.name for f in members} == expected
    for r in m['trajectories']:
        data = tar.extractfile(r['path']).read()
        assert len(data) == r['bytes']
        assert hashlib.sha256(data).hexdigest() == r['sha256']
print('621/621 trajectory hashes verified')
PY
```

要重新执行独立运动学审计，先恢复到**新目录**。`mkdir` 不加 `-p`，避免把新副本写到已有结果中：

```bash
mkdir artifacts/level3_planning/restored_frozen_621
tar -xzf docs/evidence/level3_planning/trajectories.tar.gz \
  -C artifacts/level3_planning/restored_frozen_621
cp docs/evidence/level3_planning/*.csv \
  docs/evidence/level3_planning/environment.json \
  artifacts/level3_planning/restored_frozen_621/
OPENBLAS_NUM_THREADS=1 python scripts/audit_planning_trajectories.py \
  artifacts/level3_planning/restored_frozen_621 --expected-runs 621
```

审计从当前 checkout 的原 URDF 与规划场景读取模型；更改模型前请另建指向冻结提交的 worktree。完整耗时重跑的命令在结果报告中，硬件及调度差异会改变时间数字。独立 FK 审计不重算网格碰撞；已记录的 dense 复查使用同一 C++ validator、更密采样并关闭刚体缓存。

长期保留原始结果与本证据包。临时 ELF 探针和生成模型的临时目录已清理；没有遗留规划进程、GPU 或渲染上下文。本次基准由有限请求数与硬进程时限约束，日志为 66,014 字节；没有持续写入的后台日志。
