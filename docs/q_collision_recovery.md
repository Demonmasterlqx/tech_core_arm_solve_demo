# Q 旋转在碰撞边界附近换解

本文保留 2026-10-11 修正及当时评测。2026-10-13 同输入重跑与独立审计见 [本机复现记录](reproduction_20261013.md)，完整阅读入口见 [文档索引](README.md)。

2026-10-11。本次修复 Q 阶段最近的关节解遇到碰撞边界后直接停止、遗漏附近可行解的问题。预插入姿态优化、碰撞间距、任务约束及两次回车流程继续使用原来的规则。

## 截图对应的请求

截图中的数值面板被裁切；复现输入取自当前窗口日志的 revision 179，原始记录见 [request.json](evidence/q_collision_recovery/request.json)。核心世界坐标为 `(0.14, 0, 0.22)` m，装配坐标系 RPY 为 `(6, −51, −66)`°，七关节均为零，默认安装、5 s 预算和 4 倍仿真速度。等价 `Scenario::core` 为 `{.16, .2, .47, -66, 39, 6}`，启用 `q_sweep`、`q_best_effort`，关闭核心位姿限制。

| 后端 | 修改前 Q 正向 / 负向终点 | 修改后 | 修改后在线耗时 |
| --- | ---: | ---: | ---: |
| Baseline | +20° / −90° | +90° / −90° | 2.032 s |
| RigidCache | +21° / −90° | +90° / −90° | 1.831 s |

窗口日志中旧 RigidCache 的停止原因为 `collision_twist_328_j5_link_visual/collision_core_shell_d=0.000772`：当前关节分支的 j5 外形与核心外壳仅剩约 0.772 mm，低于要求的 1 mm 间距。这个结果只说明该分支的下一段不合格。局部调整冗余关节后，相同任务可以继续到 +90°。

修正后的两个后端生成的轨迹均包含 `0 → +90 → 0 → −90°` 的 Q 序列；**预插入末轴角度变化仍为 0°**，七关节绝对变化之和仍为 276.982°。截图请求的预插入、对准、插入、升降和 P 前缀与修改前的导出轨迹逐点完全一致。没有为了增加 Q 行程而换回大幅转动末轴的预插入姿态。

## 求解方式

Q 每前进一步，先用原来的连续 IK 求解并检查关节插值段。只有这一段因碰撞或间距不足失败时，才尝试局部换解：

1. 用当前五维位置/工具轴任务的雅可比构造两个近似零空间方向，尝试正负方向及其组合。
2. 依次采用 0.025、0.05、0.1、0.2 rad 的关节扰动，并用 IK 重新收敛到**同一个**下一步 Q 任务目标。
3. 每个替代解必须通过从已接受终点出发的完整区间检查，包括原有碰撞间距、关节限位和跟踪容差。只有通过的段才加入路径；已有前缀不会被修改。

两侧换解合计最多占用本次候选规划预算的 15%。所有尝试计入请求耗时、IK 和验证统计，并服从原有完整复验预留；最终仍检查整个前缀、正向 Q、原路返回、负向 Q 和驻留。如果换解失败，保留已经验证通过的部分行程。这个有限时间的局部搜索不保证找到所有分支中的最大范围。

该策略只在 Baseline/RigidCache、`optimize_joint_motion=true`、`q_best_effort=true` 时启用。严格 Q 请求及关闭优化的对照保持原求解方式。

## 验证结果

两后端分别完成固定 109 个请求：原 ±20 cm / ±90° 开发组的全部 64 个请求、原全部 40 位姿、4 个障碍/远距离负例，以及本次截图。成功轨迹全部关闭两个碰撞缓存后重新验收。

| 分组，每个后端分别统计 | 原成功数 → 本次成功数 | 原完整 ±90° 数 → 本次数 |
| --- | ---: | ---: |
| 扩大范围开发组 | 20/64 → 20/64 | 6 → 11 |
| 原 40 位姿 | 37/40 → 37/40 | 14 → 36 |
| 本次截图 | 1/1 → 1/1 | 0 → 1 |
| 障碍/远距离负例 | 均正确拒绝 4/4 | — |

没有原成功请求变失败，配对成功请求的 Q 双向累计行程均未减少。名义零位起步的预插入末轴仍为 28.65°；截图请求为 0°。Baseline 的开发请求 `development_027_zero` 在本次运行选到了略有不同的强制阶段路径，关节累计运动从 19.953 降至 19.746 rad，Q 均为 0°；有时间上限的预插入搜索不保证不同运行之间所有候选完全相同。

本次没有重跑先前的 200 请求扩展评测组；[旧大范围报告](planner_quality.md) 的该组结果仍只对应提交 `8859a1c`，不能作为本次版本的新结果。

- [逐例与配对汇总](evidence/q_collision_recovery/summary.json)：包含所有失败与 Q 部分行程。
- [独立 URDF 审计](evidence/q_collision_recovery/urdf_audit.json)：218 条记录、零错误。116 条成功轨迹按在线采样数的两倍检查 URDF FK、机构任务、关节范围、时序及解析速度/加速度；102 条失败记录只检查有限字段和关节范围。网格碰撞仍由 C++ 无缓存验证器检查。
- [CTest](evidence/q_collision_recovery/ctest.log)：24/24 通过；[新增回归输出](evidence/q_collision_recovery/contract.log) 包括本次位姿的两后端、零起点、小预插入末轴、0.004 rad / 最少 4 子步的无缓存整轨迹复查、超时和障碍拒绝。原只能转一部分的对照测试仍在 +63° 停止后到达 −90°。
- [真实窗口 X11/XTest](evidence/q_collision_recovery/x11.json)：预览与执行分离、两后端、归零、核心自由调整、失败展示及执行终态均通过。[运行记录](evidence/q_collision_recovery/runtime.json) 保存软件渲染器、进程检查及二进制哈希；测试进程已退出，用户原窗口保留。

## 复现

```bash
cmake --build build -j4
CUDA_VISIBLE_DEVICES='' ctest --test-dir build --output-on-failure -j2
python3 scripts/run_planner_quality.py --suite docs/evidence/q_collision_recovery \
  --output artifacts/q-local-baseline --split all --method baseline --cpu 0 --timeout 5
python3 scripts/run_planner_quality.py --suite docs/evidence/q_collision_recovery \
  --output artifacts/q-local-rigid --split all --method rigid_cache --cpu 2 --timeout 5
```

输出目录必须新建。输入、源码及静态库、冻结二进制哈希保存在 [执行元数据](evidence/q_collision_recovery/run_manifests.json)；`git_head` 表示构建基底，未提交的修改由源码哈希标识。旧截图结果见 [对照指标](evidence/q_collision_recovery/before_screenshot.csv) 和 [对照元数据](evidence/q_collision_recovery/before_screenshot_manifests.json)。原始轨迹、实验程序、日志及独立审计调用脚本保留在本地 `artifacts/q_right_turn_20261011/`。

已重建 `build/rm26_demo`。已有窗口仍使用其启动时的二进制，退出后重新运行 `./scripts/run_planning_demo.sh` 才会载入此次修改。
