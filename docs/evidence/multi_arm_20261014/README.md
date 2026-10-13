# 2026-10-14 多机械臂验证

对应[架构接入说明](../../multi_arm.md)与[MecArm 模型](../../mec_arm.md)。代码基于 `d4114ea`，实现提交及校验值记录在 `summary.json`。未使用 subagent；当前环境没有用户指定的 “6 astra” 模型。

本目录保留首次接入时的倾斜工位结果；重放这里的历史轨迹应使用 `73d7c6c` 版本的模型。后续已将 MecArm 核心改为直立并调整挂载与起点，当前验证见[直立工位记录](../mec_arm_upright_20261014/README.md)。

## 构建与测试

- 窗口 Release：`ctest-viewer.log`，33/33。
- 无窗口 Release：`ctest-headless.log`，31/31；末轮动态维度输入校验及导入器调整后重建，`ctest-headless-incremental.log` 对相关三项复验。
- `multi_arm_contract` 覆盖 3/6/7/8 维、五种 IK、六轴/七轴 FK、控制、碰撞缓存、Demo、错误维度；`external_robot_import` 用临时的重排关节、直线/旋转混合模型验证无需改代码和重编译。
- `mec_arm_assembly` 验证 Baseline、RigidCache、无缓存两倍密度复验、负例、预览与错误轨迹维度。

## 模型与轨迹

`mec_arm_fk.json`：68 个姿态，11 个源连杆，源 URDF 和 MuJoCo 的位置、姿态与 TCP 雅可比独立对照。`reproducibility.json`：RM26 与 MecArm 共六个生成场景/报告/清单离线重建逐字节一致；MecArm 15 个源文件还由清单 SHA-256 校验。

`all_methods`：在同一个固定 MecArm `dev_nominal` 预设上，九种方法各跑一次，9/9 完整成功，全部经过 C++ 加密无缓存复验和独立 Python 源 URDF 审计。Atlas 用同一个 development 预设构建；这里验证的是接入兼容性，不是泛化性能或模型间速度比较。

`mec_suite`：RigidCache，单次 10 s，四个固定请求全部保留。名义预设成功；35° Q 请求在 `ik_twist_234_iteration limit` 失败；不可达和阻挡两个负例均拒绝。四条记录均通过独立审计。**1/4 包含两个有意失败的负例，不能作为实际应用成功率。**

两组目录都有 `runs.csv`、`scenarios.csv`、关节顺序/单位/限制映射 `robot.csv`、`audit.json`、压缩轨迹和逐轨迹 SHA-256。展开后可复验：

```bash
mkdir -p artifacts/multi-arm-audit
cp docs/evidence/multi_arm_20261014/all_methods/{runs.csv,scenarios.csv,robot.csv} artifacts/multi-arm-audit/
tar -xJf docs/evidence/multi_arm_20261014/all_methods/trajectories.tar.xz -C artifacts/multi-arm-audit
python3 scripts/audit_planning_trajectories.py artifacts/multi-arm-audit --robot mec_arm --expected-runs 9
```

## 窗口与回放

`keyboard_mec_arm.json` / `.jsonl` 是真实 X11/XTest 交互记录：六关节零位启动 → R 预设 → 规划 → 预览期间真实状态保持 → 再次 Enter 执行 → COMPLETE → 归零。软件渲染为 llvmpipe。

该窗口请求允许 Q 尽量旋转，实际验收范围为 **+22° → -65°**，必需装配阶段全部完成；未达到完整 ±90°。固定 5 s 时间预算下范围可能随运行负载变化。静态预览、完成图与双视角回放图保存在 `docs/images/mec-arm-*.png`；`replay.json` / `.log` 记录回放前的 1108 个无缓存加密检查样本及 CPU 渲染器。

MecArm 预设使用实验室倾斜工位和显式虚拟刚性转接，关闭官方位姿表门槛；转接件没有实体碰撞模型。上述证据验证软件接入、源机械臂运动学与声明场景内的轨迹，不能解释成源机械臂已具备真实装配夹具、力控制或完整官方赛场可达性。
