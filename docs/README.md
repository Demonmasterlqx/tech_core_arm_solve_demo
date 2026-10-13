# 文档索引

[项目首页](../README.md) · [方案手册](handbook/README.md) · [运行证据索引](evidence/README.md)

当前统一入口是 `arm_demo --robot ID`；六种内置选择可用 `arm_demo --list-robots` 查看。新导入的 HKUST ENG、PIE、追梦单臂和双臂左臂见[四套 CAD 机械臂](exported_arms.md)，架构见[接入指南](multi_arm.md)。之前的 **2026-10-13 本机复现**以及 2026-10-11 的评测和早期 40 位姿实验按各自版本保留。本文按用途组织阅读入口，所有运行命令默认从仓库根目录执行。

## 启动、操作与接口

| 需要做什么 | 阅读入口 |
| --- | --- |
| 接入新机械臂、选择模型、动态维度 API | [多机械臂指南](multi_arm.md) |
| HKUST ENG、PIE、追梦单臂与双臂左臂 | [四套 CAD 机械臂](exported_arms.md) |
| MecArm 源模型、装配假设和验证 | [MecArm 说明](mec_arm.md) |
| 安装依赖、构建、调用 C++ 库、排错 | [代码使用与框架](handbook/03_code_guide.md) |
| 操作联合逆解与规划窗口 `rm26_demo` | [键盘规划 Demo](planning_demo.md) |
| 控制独立七关节机械臂 `rm26_arm` | [机械臂使用说明](rm26_arm.md) |
| 运行原科技核心规则仿真 `tech_core` | [核心操作、批量运行与 API](tech_core.md) |
| 导出三份手册与独立 Git 交付包 | [打包说明](handbook/README.md#压缩包目录与复现) |

三个窗口入口的按键和仿真范围不同。联合 Demo 的流程是 **R 载入预设 → Enter 规划并预览 → 成功后再次 Enter 执行**；四套新 CAD 模型在启动时已载入各自预设，RM26/MecArm 保留零位启动。

## 理论与实现

| 文档 | 内容 |
| --- | --- |
| [逆运动学理论](handbook/01_inverse_kinematics.md) | 五维位置/工具轴任务、阻尼逆解、关节边界与冗余分支 |
| [轨迹规划理论](handbook/02_trajectory_planning.md) | 预插入候选、碰撞检查与绕行、装配跟踪、Q 范围、时间化和验收 |
| [代码使用与框架](handbook/03_code_guide.md) | 参数、坐标、模块、接口示例和维护经验 |
| [上游算法来源](planning_reference.md) | 固定参考提交、任务雅可比基线与本项目扩展的边界 |
| [联合场景审计](planning_scene_audit.md) | 固定抓持、携带物、碰撞代理与允许接触假设 |

## 最新复现与验证

最新模型接入见[四套 CAD 机械臂验证](evidence/exported_arms_20261014/README.md)：保留完整构建测试、四套固定预设的九方法轨迹、独立源 URDF 审计和两个交互窗口的记录。

此前的 RM26 评测见[2026-10-13 本机复现记录](reproduction_20261013.md)及其[精简证据](evidence/reproduction_20261013/README.md)：当时窗口构建 25/25 项、无窗口构建 23/23 项 CTest 通过，两后端各 100 请求均有 31 条有效轨迹，完整 Q 双向 ±90° 分别为 14 条和 15 条。

有效轨迹允许 Q 末段只完成通过验证的部分范围，插入、升降与 P 翻转仍须完整完成。统计保留所有失败；固定墙钟预算下的 Q 行程、耗时不能直接跨机器作性能比较。

| 记录 | 阅读方式 |
| --- | --- |
| [四套 CAD 机械臂验证](evidence/exported_arms_20261014/README.md) | 最新完整构建、固定预设轨迹、初始对齐、夹爪载物、窗口与离线重建 |
| [2026-10-13 本机复现](reproduction_20261013.md) | 当前代码评测、历史程序重跑、归档复验、CAD、视频与交付检查分别列出 |
| [联合 Demo 交互验证](planning_demo_validation.md) | 最新窗口复现入口及 2026-10-11 零位、预览、接近段、Q 旋转的迭代记录 |
| [首次文档与代码交付验证](delivery_validation.md) | `0fbd7f7` 交付阶段的构建、小样本和打包检查；早于本机完整复现 |
| [独立机械臂验证](rm26_arm_validation.md) | 原独立控制与 FK 的验证记录；当前环境复验见本机复现 |
| [核心规则仿真验证](validation.md) | 原几何、规则与渲染的验证记录；当前环境复验见本机复现 |

## 历史评测与视频

这些报告用于比较特定修改前后的行为。先核对算法版本、固定输入、安装外参、起点、时间预算和成功定义，再比较数字。原始结果与失败原因保持在对应报告中。

| 报告 | 协议与阶段 |
| --- | --- |
| [两后端各 100 请求](planner_success100.md) | 2026-10-11，`fd6e983`；有效轨迹均 31/100，完整 Q 为 19/100、18/100；同输入本机复现为 14/100、15/100 |
| [预插入动作与规划质量](planner_quality.md) | `8859a1c`，开发组、扩展组、旧 40 位姿和负例；条件成功率不替代整体成功率 |
| [Q 碰撞边界换解](q_collision_recovery.md) | Q 局部冗余换解修正及其单独评测 |
| [九方法冻结评测](planning_results.md) / [源码审查](planning_review.md) | 早期 621 次请求；算法行为、缓存策略与耗时均属于当时版本 |
| [40 位姿离线搜索](pose40_q_sweep.md) | 离线选择起点，严格 Q 往返；搜索耗时与选中轨迹耗时分别记录 |
| [40 位姿在线严格 Q](pose40_online.md) | 固定实际起点、3 s 总预算，历史 20/40；本机历史程序重跑为 14/40 |
| [40 位姿尽量旋转 Q](pose40_q_range.md) | 旧 best-effort 版本，完整与部分 Q 分开；本机旧程序中断及当前代码重跑见复现报告 |
| [单场景视频](planning_video.md) | 安装间距、基座高度、倾斜与运动倍率实验 |

上述视频页保留原始生成协议。2026-10-13 重新渲染的 8 段视频及媒体检查见[复现记录](reproduction_20261013.md#完整视频与媒体验证)，视频回放归档轨迹，不代表当日重新规划的统计结果。

## 规则、模型与几何来源

| 文档 | 内容 |
| --- | --- |
| [规则追溯](rules.md) | 手册页码、尺寸、装配顺序与建模假设 |
| [核心模型来源](fidelity_sources.md) | HKUST 网格、官方 CAD、坐标与许可来源 |
| [核心尺寸审计](fidelity_geometry_audit.md) | MARK、外形、插接配合与尺寸核对 |
| [能量单元 CAD](energy_cad.md) | 官方外形、孔接口修订及离线验证 |
| [RM26 机械臂模型](rm26_arm_model.md) | 上游 URDF、关节定义、网格与 FK 一致性 |
| [MuJoCo 依赖来源](../reference/mujoco/README.md) | 固定版本、下载校验与许可 |

## 文档与证据维护

`handbook/` 保持三份主文档与一个索引，供打包脚本导出 Markdown、PDF 和离线 HTML；操作细节按程序入口分文档。历史报告继续使用原路径，便于旧链接、脚本和证据清单追溯。

新实测应注明日期、源版本、输入和预算，保存完整分母、失败原因及验证方式；旧报告增加后续记录链接，原测量值不覆盖。可移植的精简数据放入 `docs/evidence/`，大型轨迹、视频和本机日志放入被 Git 忽略的 `artifacts/`。指向 `artifacts/` 的链接只在保留对应输出的机器上可用，源码交付包不包含这些大文件。

历史实施计划位于 [科技核心计划](../plan/technology-core/plan.md)、[保真计划](../plan/technology-core/fidelity.md)、[机械臂计划](../plan/rm26-arm/plan.md)和[三级规划计划](../plan/level3-planning/plan.md)；当前操作以本索引链接的使用文档为准。
