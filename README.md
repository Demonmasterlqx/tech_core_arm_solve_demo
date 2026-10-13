# RoboMaster 2026 科技核心与机械臂仿真

C++17 + MuJoCo 3.3.7，统一支持 HKUST RM26 七轴机械臂、MecArm 六轴机械臂和外部 URDF 模型包。推荐入口 `arm_demo --robot mec_arm`（或 `--robot rm26`）完成场景编辑、逆运动学、碰撞检查、轨迹预览与装配仿真；原 RM26 命令保留。

[文档索引](docs/README.md) · [方案手册](docs/handbook/README.md) · [多机械臂接入指南](docs/multi_arm.md) · [MecArm 模型与验证](docs/mec_arm.md)

## 启动联合 Demo

已构建的环境直接运行：

```bash
./scripts/run_planning_demo.sh --robot mec_arm
# 原 RM26：./scripts/run_planning_demo.sh --robot rm26
```

启动于当前模型的关节零位（受关节边界约束）。按 **R** 载入装配预设，按 **Enter** 规划并预览，预览成功后再次 **Enter** 执行；也可以从零位直接规划。

| 操作 | 按键 |
| --- | --- |
| 机械臂末端 X / Y / Z | W、S / A、D / Q、E |
| 核心 X / Y / Z | I、K / J、L / U、O |
| 核心 roll / pitch / yaw | Shift 配合上述核心按键 |
| Baseline / RigidCache（默认） | 1 / 2 |
| 关节归零 / 暂停继续 / 退出 | Backspace / Space / Esc |

脚本使用 CPU 与 Mesa/llvmpipe 软件渲染。交互需要可见桌面 `DISPLAY`；无显示时自动使用不可见的 Xvfb，检查时应加 `--duration 2`。完整参数、鼠标操作与预览规则见 [键盘规划 Demo](docs/planning_demo.md)。

![联合规划的成功虚影预览](docs/images/planning-demo-preview.png)

## 首次构建

自动依赖脚本面向 Linux x86_64。Ubuntu / Debian 安装系统依赖后，在仓库根目录执行：

```bash
sudo apt-get install build-essential cmake curl ca-certificates \
  libeigen3-dev libglfw3-dev libgl1-mesa-dev libpng-dev \
  mesa-utils xvfb xauth libxtst6
./scripts/bootstrap_mujoco.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
CUDA_VISIBLE_DEVICES='' ctest --test-dir build --output-on-failure
./scripts/run_planning_demo.sh
```

正常 C++ 构建运行不需要 Python，模型与网格已入库；检测到 Python 时 CTest 还会运行外部机器人包导入测试。MuJoCo 自动安装至 `.deps/mujoco-3.3.7` 并校验下载包；来源见 [依赖说明](reference/mujoco/README.md)。自定义 MuJoCo 路径、无窗口构建、C++ 库调用和排错见 [代码手册](docs/handbook/03_code_guide.md)。

## 程序与阅读入口

| 入口 | 用途 | 文档 |
| --- | --- | --- |
| `arm_demo` / `rm26_demo` | 联合逆解、碰撞检查、规划预览与执行 | [操作说明](docs/planning_demo.md) |
| `arm_plan` / `arm_plan_example` | 命令行规划与最小库调用示例 | [代码手册](docs/handbook/03_code_guide.md) |
| `arm_control` / `rm26_arm` | 选定机械臂的关节控制与末端平移 | [机械臂说明](docs/rm26_arm.md) |
| `arm_kinematics_example ROBOT` | 不依赖装配场景的 FK/IK 示例 | [接入指南](docs/multi_arm.md) |
| `tech_core` | 已对准单元的导向机构与一至四级规则仿真 | [核心仿真说明](docs/tech_core.md) |

理论分为 [逆运动学](docs/handbook/01_inverse_kinematics.md) 和 [轨迹规划](docs/handbook/02_trajectory_planning.md)。规则出处、模型来源、历史实验、证据和交付导出统一由 [文档索引](docs/README.md) 导航。

## 最近验证与范围

[多机械臂验证](docs/evidence/multi_arm_20261014/README.md)覆盖动态维度求解器、RM26 回归、MecArm 源 URDF 对照、装配轨迹与独立审计、窗口交互，以及无需重新编译的外部机器人接入。MecArm 使用明确声明的虚拟转接夹具和实验室核心位姿，详见[模型说明](docs/mec_arm.md)。

[2026-10-13 本机复现](docs/reproduction_20261013.md)记录窗口构建 **25/25**、无窗口构建 **23/23** 项 CTest 通过。沿用相同的固定 100 请求、每次 5 s 预算：

| 指标 | Baseline | RigidCache |
| --- | ---: | ---: |
| 有效轨迹，允许部分 Q 行程 | 31/100 | 31/100 |
| 完整 Q 双向 ±90° | 14/100 | 15/100 |

200 条记录通过独立 URDF 审计，62 条成功轨迹通过 C++ 无缓存验收。所有失败保留在分母；有效轨迹要求插入、升降与 P 翻转等必需阶段完整通过。输入为固定样本，不能推广为任意位姿成功率。

[2026-10-11 历史评测](docs/planner_success100.md)的完整 Q 为 19/100、18/100。固定墙钟预算下的候选选择和 Q 范围受运行环境影响，两批结果分别保留；更多协议见 [文档索引](docs/README.md)。

联合 Demo 使用固定抓持的能量单元与运动学回放，尚未包含自由抓取、视觉、实体机械臂或接触力控制。碰撞验证依赖代理几何和离散采样；安装、抓持外参仍需实物标定。原 `tech_core` 导向机构与独立 `rm26_arm` 的范围分别见各自说明。

本机输出位于 Git 忽略的 `artifacts/`；可随仓库分发的精简输入、结果和校验信息见 [证据索引](docs/evidence/README.md)。
