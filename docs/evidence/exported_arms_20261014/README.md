# 2026-10-14 四套导出机械臂验证

对应[模型与使用说明](../../exported_arms.md)。基于 `eeeef31` 接入 HKUST ENG、PIE ENG、追梦单臂，以及追梦双臂的左臂；实现提交、模型和程序校验值见 [summary.json](summary.json)。本次未使用 subagent。

在源目录的 `finished` 出现后才开始复制和接入。原始包的 187 个文件、47 个 STL 保持字节不变，校验清单位于 `reference/exported_arms/checksums.json`。双臂运行场景仅含左臂七轴，躯干固定零位，夹爪固定抓持能量单元。

## 完整构建与模型检查

| 检查 | 结果与记录 |
| --- | --- |
| Release，带窗口 | **46/46**，`ctest-viewer.log` / `.xml` |
| Release，无窗口 | **44/44**，`ctest-headless.log` / `.xml` |
| 原始 URDF 对照 | 每模型 68 个操作起点、零位、边界、随机姿态，见 `*_fk.json` |
| 离线完整重建 | **1,652 个生成文件逐字节一致**，见 `reproducibility.json` |

`exported_arm_test` 包括五种 IK、控制、固定夹爪和单臂选择、能量单元挂载、核心直立及初始对齐、Baseline/RigidCache/BoundedQP 装配、加密无缓存回放，以及障碍物和不可达目标拒绝。`exported_robot_assets` 验证原包与所有生成依赖的 SHA-256，并拒绝非法复合关节碰撞排除和启动标志。

FK 验证直接读取未修改的原始 URDF，应用明确保存的辅助关节固定值，再对照所有保留连杆与 TCP。四套模型的位置最大绝对误差不超过 4.45e-16 m，差分雅可比误差不超过 3.30e-10。这里验证坐标与运动学转换的一致性，不代表实物标定精度。

`ctest-final-contracts.*` 和 `ctest-final-headless-metadata.*` 保留接入过程中针对元数据的补充检查；最终结果以上表两次完整构建为准。

## 九种方法与轨迹

四套模型各自固定的 `dev_nominal` 预设上，九种方法各运行一次，单次预算 10 s，均完成完整装配并通过 C++ 加密无缓存复验和独立 Python 源 URDF 审计：

| 模型 | 接受的轨迹 | 独立审计 |
| --- | ---: | ---: |
| `hkust_eng` | 9/9 | 9/9 |
| `pie_eng` | 9/9 | 9/9 |
| `zhuimeng_single` | 9/9 | 9/9 |
| `zhuimeng_dual` | 9/9 | 9/9 |

这些名义请求的 Q 目标为 **0°**；联合窗口的双向 Q 行程单独记录在下一节。Atlas 仅用同一 development 预设建库，其结果用于验证接入兼容性。36 条固定请求的结果不能解释成任意核心位姿成功率或泛化性能；耗时也不用于模型间比较。

每个模型子目录保存 `runs.csv`、`scenarios.csv`、`robot.csv`、`atlas_build.csv`、`audit.json`、完整压缩轨迹及逐文件 SHA-256。归档已逐项回读校验。独立审计检查源运动学、抓持外参、机构顺序和时序；网格碰撞由 C++ 验证器检查。恢复后可复验：

```bash
robot_id=zhuimeng_single
mkdir -p artifacts/exported-replay/$robot_id
cp docs/evidence/exported_arms_20261014/$robot_id/{runs.csv,scenarios.csv,robot.csv} artifacts/exported-replay/$robot_id/
tar -xJf docs/evidence/exported_arms_20261014/$robot_id/trajectories.tar.xz -C artifacts/exported-replay/$robot_id
python3 scripts/audit_planning_trajectories.py artifacts/exported-replay/$robot_id --robot "$robot_id" --expected-runs 9
```

`precision-regression-before.csv` 保留修复前的追梦单臂诊断记录：五种方法在翻转插值段超过 0.1 mm 接触边界，Atlas 因没有成功开发轨迹而失败。仅提高 IK 端点精度不足以修复关节插值误差；最终所有后端共享中间段加点，碰撞阈值保持原值。修复后的九条记录见 `zhuimeng_single/runs.csv`。

## 实际窗口与回放

四套独立控制窗口的 `control.json` / `.log` 均通过：十个方向按键、释放后保持、暂停恢复、末关节位置与速度模式，以及复位。

四套联合窗口的 `keyboard.json` / `.jsonl` / `.log` 均通过真实 X11/XTest 操作：启动等于本模型装配预设 → 规划 → 虚影预览且真实机械臂不移动 → 再次 Enter 执行 → COMPLETE → Backspace 主动关节归零。软件渲染器为 Mesa llvmpipe。

| 模型 | 实际 Q 先正转、再反转至 | 完整 ±90° |
| --- | --- | --- |
| HKUST ENG | +90° → −43° | 否 |
| PIE ENG | +26° → −90° | 否 |
| 追梦单臂 | +55° → −90° | 否 |
| 追梦双臂左臂 | +19° → −90° | 否 |

Q 使用现有 demo 的尽量旋转模式；必需的插入、抬升和 P 翻转阶段全部完成。上表为本次固定 5 s 在线预算下的观测值，行程与停止原因以交互报告为准，不承诺每次运行相同。

[四套预设截图](../../images/exported-arms-presets.png)来自实际窗口，[回放截图](../../images/exported-arms-replay.png)同时展示全臂和夹爪近景。每个模型的 `replay.json` / `.log` 记录最终回放程序的单帧输出、原轨迹和模型哈希、CPU 渲染器及回放前的无缓存加密检查。回放相机沿用模型声明的正面方向，速度和加速度记录按各关节的 0.6 rad/s、1.2 rad/s² 仿真设置输出。

原包不提供实体质量、惯量或驱动参数；关节范围是导出查看器范围，PIE 还缺少七处上游几何引用。本项目的固定抓持、数值惯量与预设工作空间用于运动学仿真，具体假设和来源条款见[模型说明](../../exported_arms.md)。
