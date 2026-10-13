# 多机械臂架构与接入

项目现在使用同一套控制、运动学、规划和显示代码加载不同机械臂。内置 `rm26`、`mec_arm`、`hkust_eng`、`pie_eng`、`zhuimeng_single` 和只启用左臂的 `zhuimeng_dual`；新增四套 CAD 模型的来源、夹爪外参与使用方法见[模型说明](exported_arms.md)。其他固定基座机械臂通过模型包目录接入，无需增加 C++ 类型或重新编译。原 `rm26_arm`、`rm26_plan`、`rm26_demo` 命令和 CMake 链接名保留，默认仍为 RM26。

## 构建与使用

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure

./build/arm_control --list-robots
./build/arm_control --robot mec_arm --headless --demo --duration 1
./build/arm_kinematics_example mec_arm
./scripts/run_arm_software_viewer.sh --robot mec_arm
./scripts/run_planning_demo.sh --robot mec_arm
```

独立窗口支持末端 XYZ 平移、位置/速度模式、数字键选关节及 `[` / `]` 遍历关节。联合窗口支持编辑核心位姿、规划、虚影预览及验证后执行。四套新 CAD 模型默认启动于装配预设，RM26/MecArm 保留受边界约束的零位启动。**R** 载入当前机器人的装配预设，**Enter** 规划，成功后再次 **Enter** 执行。MecArm 预设是实验室装配场景，几何外参与范围见[模型说明](mec_arm.md)。

无窗口构建：

```bash
cmake -S . -B build-headless -DTECH_CORE_VIEWER=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build-headless -j4
./build-headless/arm_control --robot mec_arm --headless --duration .2
```

命令行规划、轨迹审计和单帧回放：

```bash
./build/arm_plan --robot mec_arm --list
./build/arm_plan --robot mec_arm --case dev_nominal --method rigid_cache \
  --timeout 10 --dense-check --output artifacts/mec-plan
python3 scripts/audit_planning_trajectories.py artifacts/mec-plan \
  --robot mec_arm --expected-runs 1
python3 scripts/render_planning_video.py artifacts/mec-plan --build-dir build \
  --case dev_nominal --method rigid_cache --still 0 --output artifacts/mec-plan.png
```

审计需要 NumPy；回放还需要 OSMesa 和 FFmpeg。输出目录必须是新的，失败运行也写入 `runs.csv`。基准启动器 `run_planning_benchmark.py` 支持 `--robot`，固定历史 RM26 质量集和 40 位姿实验脚本继续表达各自的 RM26 评测协议，不自动转换成新机器人的成功率。

## 模块边界

| 层 | CMake target / 接口 | 职责 |
| --- | --- | --- |
| 模型 | `tech_core_robot` / `RobotModel` | 从 MJCF 读取机器人标识、关节顺序、速度、根节点、TCP、预设、碰撞声明；校验模型契约 |
| 控制 | `tech_core_arm` / `ArmSimulation` | 按显式关节顺序处理位置、速度、末端平移；直接更新运动学状态 |
| 运动学 | `tech_core_kinematics` / `ArmKinematics`、`solve_ik` | 独立 FK、工具点雅可比、五种 IK；不依赖科技核心、载物模型或装配场景 |
| 任务 | `tech_core_planning` / `PlanningScene`、`plan` | 连接路径、装配任务、碰撞、规则验收、时间化及轨迹验证 |
| 演示 | `tech_core_planning_demo` / `PlanningDemo` | 选中模型的显示状态、异步规划、预览、受验收门控的执行 |

`JointVector` 为 `Eigen::VectorXd`，`Jacobian` 为 `6×N`；所有求解器、搜索、缓存键、输出和播放根据模型维度工作。限位来自 MJCF 中保留的 URDF；控制速率来自模型元数据。时间化按每关节速度上限，以及数值为速度上限两倍的加速度假设计算，再乘演示速率因子；这些是仿真限制。

RM26 的末关节姿态偏好及冗余恢复顺序由模型声明，MecArm 未声明末关节锁定。直线关节不会进行 `2π` 等价角度恢复。

## C++ 调用

```cpp
#include "tech_core/arm_kinematics.hpp"
#include "tech_core/planning.hpp"

tech_core::ArmKinematics arm(tech_core::robot_model_path("mec_arm"));
Eigen::Vector3d p;
Eigen::Matrix3d r;
tech_core::planning::Jacobian j;
arm.forward(arm.home(), p, r, &j); // j.cols() == 6
tech_core::planning::TaskTarget target;
target.position = p;
target.direction = r.col(2);    // 位置 + 工具轴方向，共五维；绕轴转角自由
const auto ik = arm.inverse(target, arm.home());

tech_core::planning::PlanningScene scene(tech_core::robot_model_path("mec_arm", true));
auto request = scene.default_scenario(); // 必须沿用当前模型和正确维度的起点
tech_core::planning::PlanOptions options;
options.method = tech_core::planning::Method::RigidCache;
const auto result = tech_core::planning::plan(scene, request, options);
```

动态向量需显式指定长度，例如 `JointVector::Zero(scene.dof())`；原来依赖固定七维的 `JointVector q; q << ...` 要先分配大小。旧 RM26 `Scenario{}` 和 `JointLimits{}` 默认值保留用于现有调用；新接入代码使用 `scene.default_scenario()`、`scene.limits()`，独立合成求解器使用 `JointLimits(n)`。

## 接入其他机械臂

创建目录，放入源 URDF、网格和 `robot.json`：

```json
{
  "schema_version": 1,
  "id": "my_arm",
  "urdf": "source.urdf",
  "root_body": "base_link",
  "tool_link": "tcp_link",
  "joint_names": ["shoulder", "elbow", "wrist"],
  "home": [0, 0, 0],
  "planning_home": [0, 0, 0],
  "velocity_limits": [0.5, 0.5, 0.5]
}
```

`joint_names` 定义应用命令/CSV 顺序，可与 URDF 树顺序不同。URDF 的轴、关节 origin、固定中间节点、限位和网格变换都保留。单位为米、弧度、秒，旋转姿态采用 URDF 的 RPY；MJCF 输出为显式四元数。

```bash
python3 scripts/import_robot.py --config /path/to/my_arm/robot.json
./build/arm_control --robot /path/to/my_arm --headless --duration 1
./build/arm_kinematics_example /path/to/my_arm
```

导入器支持有限位的 revolute/prismatic，以及 fixed、mesh/box/cylinder/sphere。连续无界关节、free/ball、mimic 和未声明的活动关节会报错。网格须在模型包中，`package://.../meshes/foo.STL` 映射到包的 `meshes/foo.STL`；同名不同源网格需先消除歧义。质量与惯量使用明确的数值占位，当前不模拟力矩驱动。

只用控制和 IK 时无需 `carried_unit`。接入科技核心任务时，参照 [MecArm 配置](../models/mec_arm/robot.json) 增加：

- `carried_unit`：父连杆、相对位置、四元数及抓持/转接假设；不能把几何连接当成已验证的实体抓持。
- `mount_bodies`：和地基直接连接、允许与 `mount_visual` 重合的源连杆。
- `scenario`：`core_pose`（x/y/z/theta/phi/alpha）、`assembly_translation`、`assembly_yaw`（弧度）、目标 Q 角（度）、是否检查官方位姿表；装配阶段与碰撞验收一直保留。
- `planning_home`：当前任务的实际起点。新机器人需要重新验证可达工作空间和碰撞。
- `carried_unit_in_control`：为 true 时，独立控制场景也加载固定载物。
- `demo_start_at_preset`、`demo_camera`：选择预设启动及默认相机的方位角/俯仰角（度）。未声明时保留旧模型的零位启动与视角。
- `compound_joint_bodies`：可选的复合关节壳体对，仅接受跨恰好两个相邻运动关节的祖先/后代关系；需要说明源机构连接依据。

生成器会生成 `scene.xml` 与可选的 `planning.xml`，重用已审计的科技核心和空心单元任务几何。源码网格保留；每个 URDF collision 网格默认使用 MuJoCo 凸包，无 collision 时保守地使用 visual。相邻运动簇、同一焊接簇的自碰撞不检查；其余机器人自碰撞、机器人与单元/核心/固定物的碰撞均检查。凹结构可能需要像 RM26 那样另外分解，并在 `robot_collision_geoms` 中明确声明。

## MJCF 运行时契约与复现

`<custom>` 包含 `robot_id`、`robot_root`、`robot_tool`、`robot_joints` 文本，以及 `robot_velocity_limits`、`robot_planning_home` 数值。选中的子树内所有活动关节都必须声明，工具必须属于该子树，根节点祖先必须固定；模型需有 `home` keyframe。规划模型还必须声明 `robot_collision_geoms`，以及可选的 `robot_mount_bodies` / `robot_grasp_geoms`。抓持接触例外须通过原有的整块外壁证书，不能任意标记几何来跳过碰撞。

运行时无需读取 JSON；`--model path.xml` 可直接加载带上述元数据的 MJCF，不能与 `--robot` 同时指定。未知模型、命令维度、限位、预设、未声明关节和缺失几何会显式报错。模型路径也写入场景 CSV，回放优先解析可搬迁的内置 robot id。

```bash
python3 scripts/import_robot.py                 # 离线重建 MecArm
python3 scripts/import_robot.py --download      # 仅补齐缺失的固定版本源文件，校验 SHA-256
python3 scripts/verify_robot.py --probe build/robot_probe
python3 scripts/audit_planning_trajectories.py --robot mec_arm --self-test
```

`verify_robot.py` 独立重建源 URDF 的所有连杆 FK，用 68 个限位/随机姿态对照 MuJoCo，并用中心差分验证 TCP 雅可比。`robot_import_test.py` 在临时目录生成不同名称、混合直线/旋转、关节顺序不同的外部机器人，以真实命令行确认不需要改代码或重新编译。
