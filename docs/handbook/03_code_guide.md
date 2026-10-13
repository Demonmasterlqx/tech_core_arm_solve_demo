# 代码使用与框架说明

整理日期：2026-10-13。本文的历史评测日期按对应报告标注，整理本身未重新执行 100 请求评测。

本文对应当前采用的 RM26 七关节机械臂与科技核心联合规划方案，重点说明构建、操作、接口、模块关系和验证方法。逆解与轨迹规划的数学推导分别见同目录的 [逆解理论](01_inverse_kinematics.md) 和 [轨迹规划理论](02_trajectory_planning.md)。三份主文档在仓库中集中于 `docs/handbook/`。下文所有命令在交付包中的 Git 仓库根目录执行；路径均为仓库相对路径。

## 1. 使用范围与入口

当前推荐入口是 `rm26_demo`：编辑场景，计算轨迹，预览成功或失败候选，再明确执行成功轨迹。当前交互后端为 Baseline 和 RigidCache，默认 RigidCache。仓库还保留了独立机械臂控制和科技核心规则仿真入口，方便检查模型和规则，但三个窗口程序的按键不同。

| 入口 | 用途 | 是否需要窗口 |
| --- | --- | --- |
| `rm26_demo` | 联合机械臂、携带单元、科技核心的规划预览与执行 | 是 |
| `rm26_plan` | 命令行规划、固定场景基准、CSV 导出 | 否 |
| `rm26_plan_example` | 最小库调用示例，零位起步、规划并复验，不执行 | 否 |
| `planner_quality_benchmark` | 固定 CSV 请求集的评测程序，由 Python 包装器调用 | 否 |
| `rm26_arm` | 固定底座七关节机械臂 mock 控制、模型与 FK 检查 | 可选 |
| `tech_core` | 核心导向机构与一至四级装配规则仿真 | 可选 |
| `scripts/run_planner_quality.py` | 固定请求集、当前两个后端的可复现评测 | 否 |

“执行”是运动学仿真：按规划时间戳更新机械臂及核心机构状态，没有连接实体机械臂、驱动器、ROS 控制器、视觉或力控制。规划默认假定能量单元已通过固定抓持变换夹住，不包含取物和夹爪闭合规划。

## 2. 环境与构建

### 2.1 依赖

自动依赖安装脚本面向 Linux x86_64。代码使用 C++17、CMake 3.20 以上、MuJoCo 3.3.7 和 Eigen3 3.3 以上。窗口依赖 GLFW、OpenGL、libpng；软件渲染和 X11 验证使用 Mesa、Xvfb、XTest。

Ubuntu / Debian 示例：

```bash
sudo apt-get install build-essential cmake curl ca-certificates \
  libeigen3-dev libglfw3-dev libgl1-mesa-dev libpng-dev \
  mesa-utils xvfb xauth libxtst6 python3 python3-numpy
```

正常 C++ 构建与运行不需要 Python。评测包装脚本使用 Python 3.9 以上及其标准库，独立 URDF 审计额外使用 NumPy；绘图、CAD 处理、Blender 导出属于可选离线工具，不是运行 Demo 的前提。

### 2.2 完整构建

```bash
./scripts/bootstrap_mujoco.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
CUDA_VISIBLE_DEVICES='' ctest --test-dir build --output-on-failure
```

脚本下载官方 MuJoCo 3.3.7，校验预定 SHA-256，再安装到 `.deps/mujoco-3.3.7`；下载临时目录在结束时清理。已自行安装 MuJoCo 时可配置：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_ROOT=/absolute/path/to/mujoco
```

模型与网格已经随仓库提供，首次使用无需运行模型生成脚本。默认模型路径在构建时写入目标；将仓库搬到新路径或从压缩包解压到其他机器后，应重新配置并构建，不直接复用旧的 `build` 目录或二进制。

### 2.3 无窗口构建

```bash
cmake -S . -B build-headless -DCMAKE_BUILD_TYPE=Release \
  -DTECH_CORE_VIEWER=OFF
cmake --build build-headless -j4
CUDA_VISIBLE_DEVICES='' ctest --test-dir build-headless --output-on-failure
./build-headless/rm26_plan --list
```

关闭窗口选项后，不要求 GLFW/OpenGL/libpng，也不生成 `rm26_demo`；规划库、异步 Demo 控制器及相关非窗口测试仍保留。

## 3. 联合 Demo 操作

### 3.1 启动

```bash
./scripts/run_planning_demo.sh
```

脚本将计算保持在 CPU，并明确设置 Mesa/llvmpipe 软件渲染。使用自定义构建目录时：

```bash
RM26_DEMO_BINARY="$PWD/build-custom/rm26_demo" \
  ./scripts/run_planning_demo.sh
```

需要可见桌面 `DISPLAY` 才能手动操作。没有 `DISPLAY` 时脚本创建不可见的 Xvfb 窗口，适合有限时长检查：

```bash
./scripts/run_planning_demo.sh --duration 2 \
  --screenshot artifacts/demo-check/initial.png
```

直接启动 `./build/rm26_demo` 会使用系统默认 OpenGL 设备；共享机器优先使用软件渲染脚本。启动输出会显示实际渲染器。

### 3.2 按键与坐标

| 功能 | 按键 |
| --- | --- |
| 机械臂末端 X 正 / 负 | W / S |
| 机械臂末端 Y 正 / 负 | A / D |
| 机械臂末端 Z 正 / 负 | Q / E |
| 核心 X、Y、Z 正 / 负 | I / K、J / L、U / O |
| 核心 roll、pitch、yaw 正 / 负 | Shift + I / K、Shift + J / L、Shift + U / O |
| Baseline / RigidCache | 1 / 2 |
| 规划并预览 / 执行成功轨迹 | 第一次 Enter / 再次 Enter |
| 七个关节归零，装配进度清零 | Backspace |
| 载入装配预设姿态及核心位姿 | R |
| 暂停 / 继续预览或执行 | Space |
| 向前 / 向后查看虚影 | `[` / `]`，每次约 5% |
| 相机旋转 / 平移 / 缩放 | 鼠标左键 / 右键 / 滚轮 |
| 恢复相机 / 截图 / 退出 | F / F12 / Esc |

平移使用世界坐标，机械臂控制点为携带单元开口中心 `planning_tcp`，手动移动时保持开始操作时的朝向。默认机械臂平移速度 0.06 m/s，核心平移速度 0.20 m/s，核心角速度 60°/s。

核心 XYZ/RPY 不设编辑限位，角度可连续累加。RPY 是相对装配坐标系的 roll/pitch/yaw；默认装配坐标与世界坐标并不重合。Core HUD 的世界坐标与 `Scenario::core` 中的装配坐标不能直接混填，接口约定见第 5 节。

启动和 Backspace 都将上游 URDF 的七个主关节角精确置为零。上游模型自带固定关节安装变换，零角度姿态不一定伸直。Backspace 保留当前核心位姿与规划后端；R 恢复非零装配预设，两者含义不同。Backspace 是立即复位仿真状态，不是一条经过碰撞规划的回零轨迹。

### 3.3 预览与执行约定

1. 修改场景后按 Enter。后台线程接收请求快照并规划，实体机械臂保持不动。
2. 成功后用绿色虚影回放；再次 Enter 才按轨迹原始时间戳开始仿真执行。
3. 失败后用红色虚影显示可返回的候选或前缀，失败结果禁止执行。若因起点碰撞等原因没有关节路径，仍显示请求路径及起点。
4. 黄色虚线是期望任务路径，绿色或红色虚影是实际求出的候选；两者不能混同。
5. 移动场景、切换后端、Backspace 或 R 会让原预览失效；晚返回的后台旧结果不会覆盖新场景。
6. 计算期间额外 Enter 或长按 Enter 不会排队触发执行。执行中失焦会暂停，恢复焦点后需 Space 继续。

虚影默认在约 6 秒内浏览整条路径，末尾停留后循环；该播放时钟不影响实体执行速度。执行中场景编辑锁定，Backspace 或 R 返回可编辑状态。

Q 末段采用“向两侧尽量旋转”。面板的 `Validated Q` 显示本次通过验证的实际范围；部分 Q 行程仍可能是有效轨迹。`travel_limit` 表示达到指定机构行程，`online_budget` 表示预算耗尽，其余原因保留 IK 或碰撞信息；停止角度不能直接当作机械极限。

### 3.4 常用参数与日志

```bash
./scripts/run_planning_demo.sh --backend baseline --timeout 5
./scripts/run_planning_demo.sh --motion-speed-scale 1
./scripts/run_planning_demo.sh --core-speed 0.3 --core-rotation-speed 90
./build/rm26_demo --help
```

Demo 的默认预算 5 s，CLI 接受 `(0,30]` s；运动倍率默认 4，CLI 接受 `(0,4]`。速度倍率改变时间参数化，最终规则驻留仍为 0.5 s。

默认事件日志为 `artifacts/keyboard_planning_demo/viewer.log`，每 1 MiB 轮转并保留两份备份。F12 默认截图为 `artifacts/keyboard_planning_demo/screenshot.png`，会覆盖同名文件；`--log` 和 `--screenshot` 可指定其他位置。场景变化、后端、预插入关节变化、Q 范围、停止原因及状态转换均可从日志追踪。

## 4. 代码框架与数据流

### 4.1 模块职责

| 文件或目录 | 职责 |
| --- | --- |
| `include/tech_core/planning_ik.hpp`、`src/planning_ik.cpp` | 独立 IK 接口、任务雅可比、预测步及多种数值求解器 |
| `include/tech_core/planning.hpp`、`src/planning.cpp` | 请求/结果数据契约、候选选择、分阶段装配、时间参数化及完整验证 |
| `src/planning_approach.cpp` | 起点到预插入点的碰撞感知关节空间搜索、路径缩短及失败前缀 |
| `src/planning_scene.cpp` | MuJoCo 场景、七关节 FK/雅可比、机构目标、碰撞配对及缓存 |
| `include/tech_core/planning_demo.hpp`、`src/planning_demo.cpp` | 不依赖窗口的异步规划、请求失效、预览与执行状态机 |
| `src/planning_demo_main.cpp` | GLFW 输入、相机、实体/虚影显示、HUD、日志和截图 |
| `src/planning_main.cpp` | 固定内置场景的 CLI、结果表和轨迹导出 |
| `src/planner_quality_main.cpp` | CSV 请求集评测入口，原 `scripts/benchmark_planner_quality.cpp` 整理至此 |
| `examples/plan_assembly.cpp` | 可构建的最小完整规划与复验示例 |
| `src/arm_simulation.cpp`、`src/arm_main.cpp`、`src/arm_viewer.cpp` | 独立七关节机械臂 mock 控制与查看器 |
| `src/rules.cpp`、`src/simulation.cpp` | 核心规则状态机、原导向机构物理仿真 |
| `cmake/` | 机械臂、规划、Demo 的目标及测试注册 |
| `tests/` | IK、碰撞场景、接近路径、全流程契约、Demo 状态和模型验证 |
| `scripts/` | 运行包装、批量评测、独立审计、离线资产处理 |
| `models/` | 可直接运行的 MJCF、视觉网格、碰撞代理和清单 |
| `reference/` | 上游 URDF、模型来源、固定版本源代码及许可记录 |
| `docs/` | 使用文档、来源追溯、历史评测及其精简证据 |
| `plan/` | 历史实施计划，不参与运行 |
| `artifacts/` | 本机运行输出、实验轨迹、日志和视频，默认不纳入 Git |

CMake 依赖主链是：`tech_core_rules → tech_core_sim → rm26_planning → rm26_planning_demo → rm26_demo`；`rm26_plan` 直接链接 `rm26_planning`。独立机械臂库 `rm26_arm_sim` 链接 MuJoCo，不经过联合规划库。

### 4.2 规划与界面数据流

```text
用户按键
  → PlanningDemo 更新实际起点/核心位姿/后端，并递增请求版本
  → Enter 创建 Scenario + PlanOptions 的值快照
  → 后台独立 PlanningScene
  → plan：预插入候选 → 接近路径 → 装配阶段 → Q 扩展 → 时间化 → 整体验收
  → PlanResult：成功标志、原因、轨迹、实际 Q 范围、耗时/计数
  → 核对请求版本 → DemoPreview
  → 独立虚影场景回放，实体场景保持
  → 再次 Enter 且 success=true → sample_demo_trajectory → 实体场景更新
```

每个规划工作线程拥有独立 `PlanningScene`，不能在多个线程间共享同一 MuJoCo model/data。一次配置会清理相关场景缓存，缓存只用于该场景的精确重复状态或刚性配对；不是跨请求的预计算答案表。

Demo 的 `ghost_knots` 仅用于显示失败前缀，可能重新赋予时间戳。执行始终使用原 `result.knots` 和 `result.success` 门控，不能将虚影副本当作可执行轨迹。

## 5. C++ 库调用

### 5.1 请求、单位与主要参数

公开接口位于 `include/tech_core/`，规划命名空间为 `tech_core::planning`。

| 字段 / 类型 | 用途与单位 |
| --- | --- |
| `Scenario::start` | 当前七关节角，rad；不是期望末端位姿 |
| `Scenario::core` | 初始 E 点在装配坐标系中的位置 m；`theta/phi/alpha` 使用度 |
| `assembly_translation`、`assembly_yaw` | 装配坐标系到参考世界系的平移 m、绕 Z 旋转 rad |
| `arm_base_z` | 机械臂及安装座在参考世界中的额外 Z 平移 m |
| `q_sweep`、`q_best_effort` | 启用双侧 Q 扫描、允许仅完成已经验证的部分 Q 行程 |
| `target_degrees` | Q 目标角；当前双侧扫描要求为 −90° |
| `enforce_core_pose_limits` | 库默认 true；Demo 关闭规则表位姿范围和前平面筛查 |
| `PlanOptions::method` | 规划后端，当前建议 `Baseline` 或 `RigidCache` |
| `timeout_seconds` | 本次规划预算 s；库默认 30，Demo 默认 5 |
| `motion_speed_scale` | 时间参数化倍率；库默认 1，Demo 默认 4 |
| `optimize_joint_motion` | 默认 true，启用附近预插入姿态搜索及相应恢复策略 |
| `TaskTarget` | 目标位置、世界目标轴、工具本地轴；位置 m，轴为方向向量 |
| `Knot` | 七关节角 rad、机构状态、阶段、任务目标及时间 s |

旧位姿 API 与 RPY 的对应关系为：`roll = alpha`、`pitch = phi - 90°`、`yaw = theta`。世界位置由装配外参变换得到，不能将屏幕显示的世界 XYZ 直接填到 `core.x/y/z`。`MechanismState` 的 `insertion/lift` 为 m，`flip/twist` 为 rad。

两个交互后端共享候选筛选、碰撞与完整验收合同，均已使用安全的刚体配对缓存和精确状态缓存。名称 `RigidCache` 保留历史命名，不意味着 Baseline 完全不使用缓存。其余 `Method` 枚举为历史对照方案，不是键盘 1/2 的额外选项。

### 5.2 最小完整规划调用

```cpp
#include "tech_core/planning.hpp"
#include <iostream>

int main() {
    using namespace tech_core::planning;
    Scenario request = scenario_suite().front();
    request.start.setZero();
    request.enforce_core_pose_limits = false;
    request.q_sweep = true;
    request.q_best_effort = true;
    request.target_degrees = -90;

    PlanOptions options;
    options.method = Method::RigidCache;
    options.timeout_seconds = 5;
    options.motion_speed_scale = 4;

    PlanningScene scene;
    PlanResult result = plan(scene, request, options);
    std::cout << "success=" << result.success
              << " reason=" << result.reason
              << " knots=" << result.knots.size() << '\n';
    if (!result.success) {
        // result.knots 可用于失败可视化，不能执行。
        return 2;
    }
    // result.knots 是通过本次验证的完整轨迹。
    // 可送入自己的仿真层；硬件集成需要另外实现控制与标定。
    return 0;
}
```

仓库提供对应的可构建示例，默认七关节零位、名义核心位姿、best-effort Q 和 5 s 预算，只规划并复验，不执行：

```bash
cmake --build build --target rm26_plan_example -j4
CUDA_VISIBLE_DEVICES='' ./build/rm26_plan_example --backend rigid_cache
CUDA_VISIBLE_DEVICES='' ./build/rm26_plan_example --backend baseline
```

应用目标通过 CMake 链接 `rm26_planning`：

```cmake
add_executable(my_planner main.cpp)
target_link_libraries(my_planner PRIVATE rm26_planning)
```

在本仓库目标创建之后加入上述配置，或从上级项目 `add_subdirectory` 本仓库。头文件路径、Eigen、MuJoCo 与默认模型路径由目标传播；本项目目前没有独立安装后的 `find_package(tech_core)` 包接口。

### 5.3 单独使用 IK

`solve_ik` 通过回调接收 FK，不强制使用 MuJoCo。回调需要给出当前控制点的位置、旋转，以及可选的 6×7 几何雅可比。

```cpp
using namespace tech_core::planning;
PlanningScene scene;
Scenario request = scenario_suite().front();
scene.configure(request);

ForwardKinematics fk = [&scene](const JointVector& q,
    Eigen::Vector3d& p, Eigen::Matrix3d& r, Jacobian* jac) {
    scene.forward(q, p, r, jac);
};
TaskTarget target = scene.mechanism_target(MechanismState{});
IKOptions ik;
ik.method = IKMethod::Reduced5D;
ik.max_iterations = 300;
IKResult result = solve_ik(fk, target, request.start, scene.limits(), ik);
```

`IKResult::success` 只表示任务误差和数值约束达标，不能替代路径碰撞检查。`should_stop` 可接入调用方的取消或截止时间。上述调用目标是机构初始状态的装配目标；实际整段规划应调用 `plan`，不要自行把一次 IK 解当作完整任务解。

### 5.4 读取结果与失败处理

调用方至少检查 `success`、`reason`、`validation`、`q_positive_degrees`、`q_final_degrees` 和 `full_q_sweep`。`planning_seconds` 记录在线规划总耗时；IK、接近搜索、姿态筛选、碰撞验证计数用于定位瓶颈，但各字段不能未经确认直接相加当作互不重叠时间。

失败结果可能包含从起点连通的前缀，也可能是一条被后续验证拒绝的候选，甚至为空。记录失败信息并展示，只有完整成功结果才能进入执行层。碰撞、超时、逆解失败并不直接证明几何不可达。

## 6. 测试、评测和审计

### 6.1 常规测试

```bash
CUDA_VISIBLE_DEVICES='' ctest --test-dir build --output-on-failure
```

测试覆盖规则、模型几何、机械臂控制、IK、碰撞场景、接近绕行、完整规划与异步 Demo 状态。此前历史记录为完整窗口配置 24 项；本次整理新增 `planning_library_example`，完整窗口配置 25/25 项通过，见[交付复验记录](../delivery_validation.md)。测试数量随 `TECH_CORE_VIEWER` 配置不同，不能将无窗口构建缺少窗口 CLI 检查误判为遗漏。

真实窗口键盘测试另行执行，使用新输出目录：

```bash
xvfb-run -a -s '-screen 0 1440x1000x24 -nolisten tcp' \
  python3 scripts/test_planning_demo_keyboard.py \
  --binary build/rm26_demo --output artifacts/demo-x11-run
```

该脚本通过 X11/XTest 注入真实按键，检查日志、终态及截图；释放按键、回收子进程和 X11 连接。它与单元测试互补。

### 6.2 命令行场景运行

```bash
./build/rm26_plan --list
CUDA_VISIBLE_DEVICES='' ./build/rm26_plan \
  --method rigid_cache --case dev_nominal --timeout 5 \
  --dense-check --output artifacts/nominal-check
```

`rm26_plan` 使用内置 `scenario_suite` 的场景合同；它不会自动将所有场景都改为 Demo 的 best-effort Q 设置。`--list` 可列出可用方法与场景。需要复现当前 100 次评测，应使用下一节固定输入而不是此命令的默认全部场景。

输出包括 `runs.csv`、`scenarios.csv`、`trajectories/`，轨迹记录时间、阶段、机构状态、七关节和任务目标。复用已有结果目录可能被拒绝，应为每轮指定新目录。

### 6.3 当前 100 请求协议

```bash
cmake --build build --target planner_quality_benchmark -j4
python3 scripts/run_planner_quality.py \
  --suite docs/evidence/planner_success100 \
  --output artifacts/success100-baseline-new \
  --split all --method baseline --cpu 0 --timeout 5
python3 scripts/run_planner_quality.py \
  --suite docs/evidence/planner_success100 \
  --output artifacts/success100-rigid-new \
  --split all --method rigid_cache --cpu 2 --timeout 5
```

CPU 编号应选择当前进程允许使用的 CPU，可先运行 `taskset -pc $$` 检查。先执行上面的 `cmake --build`，确保 `planner_quality_benchmark` 与当前源码一致。包装脚本只复制已构建的评测程序到本轮输出目录冻结，不自动构建，避免两个后端并行运行时竞争同一构建输出；缺少程序时会报错并提示先构建。脚本关闭 CUDA、限制数值库线程、固定 CPU 亲和性，保存可复现输入/程序哈希和轮转日志。这样统一使用工程声明的 MuJoCo、Eigen 与链接设置，避免脚本手工链接与主程序配置分叉。输出目录必须尚不存在。`--build-dir` 可指定已配置并完成构建的其他目录；`--binary` 可直接复用已有冻结评测程序。

固定请求覆盖相对名义 XYZ 各 ±20 cm、RPY 各 ±90°，零位/装配预设各 50 次，两个后端使用完全相同的 100 个请求。失败保留在分母中；有效轨迹和完整 Q ±90° 分别计数。按 `docs/planner_success100.md` 记录，2026-10-11 对算法提交 `fd6e983` 的历史复测结果是两个后端均 31/100 有效，完整 Q 分别 19/100 与 18/100。该结果属于固定输入和给定预算，不能推广为任意位姿成功率，也不代表本次打包额外重测了 100 次。

完整协议和记录保存在 `docs/planner_success100.md` 与 `docs/evidence/planner_success100/`。原始大轨迹和冻结二进制位于运行机的 `artifacts`，是否包含于交付包以包清单为准。

### 6.4 独立审计

对 `rm26_plan` 标准导出，可运行：

```bash
OPENBLAS_NUM_THREADS=1 python3 scripts/audit_planning_trajectories.py \
  artifacts/nominal-check
```

审计器从 `reference/rm26_arm/source.urdf` 重新建立 NumPy FK，检查起点、关节限位、任务跟踪、时序及速度/加速度，采样数为在线验证的两倍。它不调用规划器 FK，但也不独立实现网格碰撞；碰撞复查仍由 C++ 场景验证器负责。成功记录与失败前缀分开检查，失败前缀不被认证为可执行。

## 7. 模型、参考资产和维护边界

`models/rm26_arm/scene.xml` 是独立机械臂，`models/planning/scene.xml` 是联合规划场景，`models/tech_core.xml` 是原核心导向仿真。网格使用各 XML 中的相对路径；不要只复制 XML 而遗漏对应网格目录。

上游 URDF、固定提交及校验清单在 `reference/rm26_arm/`；核心原始 Blender 与许可记录在 `reference/hkust/`；参考算法的源文件、出处及许可说明在 `reference/planning_upstream/`。它们用于追溯和复现，正常运行不需要启动上游 ROS 工程。若分发或进一步复用第三方内容，保留对应来源与许可文件，不能将上游资产视为全部采用本项目自有许可。

离线生成脚本包括 `generate_model.py`、`import_rm26_arm.py`、`generate_planning_scene.py` 和 `scripts/cad/`。改变关节、抓持或几何后，应重新生成资产、核对清单并执行 FK/场景测试，而不是仅修改界面中的姿态显示。

当前仿真边界：

- 碰撞依赖既定代理几何、允许接触配对及有限密度采样，不是连续碰撞证明。
- 预设安装/抓持外参尚未完成实物标定，模拟速度与加速度限制不是硬件额定参数。
- 手动移动、Backspace 和 R 属于场景编辑，不保证这些瞬时改变本身无碰撞。
- 核心无限位编辑不意味着机械臂能到达任意输入，关节和碰撞验证仍生效。
- `tech_core` 的原导向机构动力学与联合 Demo 的运动学回放是不同层次，不能互相充当硬件接触控制的证据。

构建输出 `.deps/`、`build*/`、实验 `artifacts/` 和 Python 缓存默认由 Git 忽略。清理时先确认实验是否需保留；不要删除 `reference/`、`models/`、输入协议或已提交的结果证据。新增长期评测应继续使用限时、轮转日志与显式子进程回收。


## 8. 本次实现与迭代经验

### 8.1 先统一模型、坐标与零位

最初“归零”容易与演示预设混淆。当前用明确的两个操作解决：Backspace 将上游 URDF 七关节角清零，R 载入装配预设。判断零位是否正确应比较上游关节原点、轴、固定变换及独立 FK，不能只看机械臂是否伸直。

核心同样存在世界坐标、装配坐标与旧位姿字段三种表达。当前保留兼容字段，但在界面和接口文档中明确 RPY 映射及安装外参。出现“输入相同而视觉不同”时，优先排查坐标系和单位，不要先调整 IK 参数。

### 8.2 成功门控与失败可视化应同时存在

没有预览会让调试混入实际状态改变；不展示失败又会掩盖失败阶段。现在成功与失败都返回可展示信息，实体执行只接受完整成功结果。独立虚影模型、请求版本号和失败副本的独立计时共同保证“能看”不等于“能执行”。这个约定应在后续硬件集成时继续保留。

### 8.3 单次 IK 达标不代表整段装配完成

直接为预插入点求一个位置正确的 IK 解，会碰到自碰、长距离关节变化，或者随后的升降/P 翻转无解。当前先搜索接近实际起点的候选，重点保留末轴角度，并对必需阶段提前筛查，再构建从真实起点出发的完整轨迹。筛查用于减少浪费，但不能替代最终验收。

名义零位起步的历史对照中，预插入末轴变化由约 118° 减到 28.6°，七关节绝对变化之和由约 574° 减到 455.6°。它们描述固定案例的姿态变化，不是全工作空间最优保证；不能通过删掉后续动作“改善”该指标。

### 8.4 接近段也必须检查碰撞

当前状态到预插入解的关节直线可能穿过环境，即使两端都安全。接近段现以同样的边验证规则检查直接连接；有障碍时搜索绕行，再用带检查的捷径缩短。失败时只展示起点侧已连通前缀，避免把目标侧搜索树误当成可执行路径。

### 8.5 端点、插值、碰撞与缓存必须一致

曾出现 IK 端点达标，但相邻关节点间的插值偏离任务曲线，侵入装配接触阈值。当前对曲率较大的段自适应加点，并确保执行与验证使用相同插值。调整容差前应检查端点与中间状态，不能以“端点足够准确”推导整段安全。

另一类数值差异来自焊接刚性配对的网格距离：几何相对关系不变，但算法受世界旋转影响。当前用固定参考姿态计算这些刚性配对，缓存与无缓存复查采用一致约定；修正几何计算比放宽碰撞阈值更可靠。

### 8.6 Q 阶段首次碰撞不是最大可转角证明

某关节分支在 Q 的 +20° 附近失去间距，不代表所有冗余解都会受阻。实际复现发现同一核心位姿可以完成 ±90°，当前因此在碰撞边界尝试局部替代 IK，并逐段重新验证。它保留已有前缀和预插入姿态，仍受同一预算约束；没有找到更好的分支也不能宣称物理极限。

Q 的完整往返比例和整条任务有效率必须分开。近期 Q 恢复策略提高了转角覆盖，但 100 请求复测中整体成功数仍为两个后端各 31；主要失败仍位于 Q 之前，尤其 P 翻转。这指向下一步应评估哪些必需阶段的分支选择造成失败，而非只继续优化末段。

### 8.7 性能优化与可验证性一起保留

两个后端共享安全合同，优化不得绕过关节限位、起点连接、装配跟踪、完整轨迹和规则验收。请求使用统一在线截止时间，候选筛查、恢复尝试、碰撞验证与重试都要占用预算，不能只计成功的最后一次调用。

部署前先固定输入集和成功判定，再优化，再做同输入配对复测。检查是否丢失原有成功请求、是否只缩短了可选 Q 行程、是否扩大容差，以及是否引入保存的逐位姿答案。冻结二进制、输入、源码摘要和逐例 CSV，比单独保留一个成功率数字更有价值。

## 9. 故障定位与验证路线

### 9.1 常见问题

| 现象或错误 | 优先检查 | 处理原则 |
| --- | --- | --- |
| 找不到 MuJoCo/Eigen | `MUJOCO_ROOT`、系统 Eigen、CMake 缓存 | 使用依赖脚本或正确路径重新配置 |
| 解压后二进制仍读旧路径 | 是否复制了旧构建目录 | 在新位置重新配置构建 |
| 窗口不出现 | `DISPLAY`、是否运行在 Xvfb | Xvfb 是不可见检查；交互需桌面显示 |
| Backspace 后模型仍折叠 | HUD 是否七关节全零、上游固定变换 | 零角度不等于直臂；用 FK 校验 |
| 按 Enter 后实体不动 | 是否处于成功预览 | 第二次 Enter 才执行，是正常合同 |
| 红色预览不能执行 | `reason`、候选是否为空 | 先排查失败原因，不能解除执行门控 |
| `collision_*` | 碰撞配对、间距、所处阶段 | 先检查起点/抓持/外参，再检查路径；不要直接关闭碰撞 |
| `ik_*` 或不下降 | 目标坐标/单位、限位、初值、阶段 | 检查数值与分支，不直接宣布不可达 |
| `online_budget` 或接近子预算耗尽 | 各阶段耗时、CPU 负载、时间预算 | 截止时间终止搜索不等于空间极限 |
| Q 未到 ±90° 但显示成功 | `q_best_effort`、两侧停止原因 | 此合同允许部分 Q；另看 `full_q_sweep` |
| 切换后端后原轨迹消失 | 请求版本与预览有效性 | 这是防止旧方案用于新请求的机制 |
| 验证通过而手动编辑发生穿插 | 是否处于 jog/复位而非规划执行 | 场景编辑与已验证轨迹执行的约束不同 |

### 9.2 建议的验证顺序

1. 核对模型来源与资产完整性；独立 URDF/MuJoCo FK、零位、控制点和抓持坐标一致。
2. 执行 IK 与场景测试，确认轴方向、关节边界、碰撞配对和阈值。
3. 对名义零位和装配预设分别检查预插入、必需装配阶段、Q 部分与完整行程。
4. 加入阻挡、不可达、起点碰撞、限时和失败前缀等负例。
5. 对成功轨迹进行无缓存密采样复查及独立 URDF 审计。
6. 固定输入后批量评测，保留所有失败，分别汇总整体成功、Q 完整率、起点类型、耗时与失败阶段。
7. 运行真实窗口回归，检查首个 Enter 只预览、第二个 Enter 执行、旧请求失效和退出资源清理。

文档、CMake 或包装脚本整理不等同于算法重评测。交付验证应分别记录“构建与测试通过”和“历史固定样本成功率”，避免将旧报告数字标注为新日期实测。

## 10. 历史报告阅读分组

仓库保留旧证据以便追溯，以下文件不是同一版算法、同一安装或同一分母的重复报告。

| 分组 | 入口 | 解释 |
| --- | --- | --- |
| 当前联合 Demo 合同 | `docs/planning_demo.md` | 键盘、零位、预览/执行、当前两后端行为 |
| 当前固定 100 请求 | `docs/planner_success100.md` | 最新已记录整体成功率；固定输入、分母、失败分类明确 |
| Q 局部换解 | `docs/q_collision_recovery.md` | 用户问题位姿、局部替代解及完整 Q 复验 |
| 大范围质量与预插入动作 | `docs/planner_quality.md` | XYZ ±20 cm/RPY ±90°、起点分组、末轴变化和条件成功率 |
| 早期方法对照 | `docs/planning_results.md`、`docs/planning_reference.md` | 多方法冻结评测和基线来源，耗时不可视作当前版本普遍保证 |
| 40 位姿迭代 | `docs/pose40_q_sweep.md`、`docs/pose40_online.md`、`docs/pose40_q_range.md` | 严格 Q、在线重跑、尽量旋转的不同阶段；安装/起点/预算需分别看协议 |
| 机械臂来源与验证 | `docs/rm26_arm_model.md`、`docs/rm26_arm_validation.md`、`docs/rm26_arm.md` | 上游模型、独立 FK、mock 控制入口 |
| 碰撞与抓持 | `docs/planning_scene_audit.md` | 携带物、抓持变换、碰撞代理和允许接触假设 |
| 科技核心规则与几何 | `docs/rules.md`、`docs/fidelity_sources.md`、`docs/fidelity_geometry_audit.md`、`docs/energy_cad.md` | 规则出处、CAD 来源、保真范围和修订假设 |
| 运行证据 | `docs/evidence/` | 与对应报告关联的输入、CSV、JSON、截图索引及校验信息 |

本次代码整理将质量评测 C++ 入口从脚本目录纳入正式源代码与 CMake 目标，提供可构建库调用示例，并集中整理文档入口；保留算法合同、模型/参考资产路径、已有测试和历史评测证据。后续修改优先维持这些可追溯接口，避免为了目录美观引入资产路径失效或使历史命令含义改变。
