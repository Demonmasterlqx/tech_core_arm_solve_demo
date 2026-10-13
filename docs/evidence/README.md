# 运行证据索引

[文档索引](../README.md) · [最新本机复现](../reproduction_20261013.md)

本目录保存可随 Git 分发的固定输入、逐例结果、审计摘要、模型校验和历史压缩轨迹。每组证据对应一次明确的实验或验证，阅读时先看报告中的版本、预算与成功定义。

## 评测与复现

| 证据 | 对应报告与用途 |
| --- | --- |
| [reproduction_20261013](reproduction_20261013/README.md) | [2026-10-13 本机复现](../reproduction_20261013.md)：当前评测、历史程序重跑、归档复验、CTest、资产与媒体校验 |
| [delivery_20261013](delivery_20261013/) | [首次交付验证](../delivery_validation.md)：`0fbd7f7` 交付阶段的构建、测试与小样本 |
| [planner_success100](planner_success100/) | [2026-10-11 固定 100 请求](../planner_success100.md)：共享输入、两后端逐例结果及审计 |
| [planner_quality](planner_quality/) | [预插入质量评测](../planner_quality.md)：开发、扩展、旧位姿、负例和对照组 |
| [q_collision_recovery](q_collision_recovery/) | [Q 换解修正](../q_collision_recovery.md)：问题位姿与修正后的验证 |
| [level3_planning](level3_planning/README.md) | [九方法冻结评测](../planning_results.md)：621 个请求、归档轨迹及哈希验证方法 |
| [pose40_q_sweep](pose40_q_sweep/README.md) | [40 位姿离线搜索](../pose40_q_sweep.md)：离线起点选择与严格 Q 往返 |
| [pose40_online](pose40_online/README.md) | [40 位姿在线严格 Q](../pose40_online.md)：固定起点和总预算 |
| [pose40_q_range](pose40_q_range/README.md) | [40 位姿尽量旋转 Q](../pose40_q_range.md)：完整、部分 Q 与停止原因 |

## 交互、模型与媒体

`keyboard_*.json` 对应联合 Demo 的交互迭代，按[交互验证记录](../planning_demo_validation.md)读取；核心与独立机械臂的运行、几何检查分别由[核心验证](../validation.md)、[机械臂验证](../rm26_arm_validation.md)及[尺寸审计](../fidelity_geometry_audit.md)引用。

[level3_video_closer](level3_video_closer/) 和 [level3_video_lowered_tilted_4x](level3_video_lowered_tilted_4x/) 保存[单场景视频](../planning_video.md)的规划与渲染元数据；40 位姿视频证据在各自协议目录中。本机重新渲染的视频检查见 [media-validation.json](reproduction_20261013/media-validation.json)。

## 使用约定

成功轨迹与失败前缀分别审计；失败前缀不能作为可执行轨迹。独立 URDF 审计检查运动学与时序，网格碰撞复验仍使用 C++ 场景验证器。归档恢复验证已有输出，重新运行规划器才产生新的在线测量。

报告和 JSON 中的绝对路径记录原运行环境，迁移后应按各目录说明将输入恢复到新目录。大型原始轨迹、冻结二进制、CAD 输入和视频通常位于本机 `artifacts/`，不因存在精简摘要就表示这些文件也随 Git 分发。新增实验使用新的输出目录，并保留日期、源版本、输入哈希及完整失败记录。
