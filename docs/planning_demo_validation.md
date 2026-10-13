# 联合规划 Demo 交互验证记录

[文档索引](README.md) · [当前操作说明](planning_demo.md)

## 2026-10-13 本机复现

完整窗口构建 25/25 项 CTest、无窗口构建 23/23 项 CTest 通过；联合 Demo 的 X11/XTest 覆盖两后端、成功和失败预览、两次回车、归零、场景编辑及执行终态。环境、测试日志和适用范围见 [本机复现记录](reproduction_20261013.md)及[精简证据](evidence/reproduction_20261013/README.md)。

## 2026-10-11 迭代记录

下列段落从操作文档归档，保留各阶段当时的行为、测试数量与测量值；当前行为以[操作说明](planning_demo.md)为准。

2026-10-11 的初版记录见 [初版运行时证据](evidence/keyboard_planning_demo.json)。归零与核心自由调节修正后的记录见 [修正验证](evidence/keyboard_zero_free_core.json)：重新构建默认 `build/rm26_demo`，23 项 CTest 全部通过；真实窗口的 X11/XTest 回归也通过，覆盖七关节精确归零、R 恢复非零初始姿态、核心以新速度越过旧范围、越界请求的失败预览，以及完整的预览/执行流程。控制器另验证 XYZ/RPY 的全部正负方向均不再限位、规划从零关节启动、旧规划结果不能覆盖归零状态，其他场景的默认范围检查仍生效。截图已更新。渲染为 llvmpipe，启动、执行和退出的 NVIDIA 进程表中均没有测试 Demo 进程，测试窗口和 Xvfb 已退出。

同日进一步将启动姿态统一到上游 URDF 零位，记录见 [上游零位与启动验证](evidence/keyboard_urdf_zero.json)：在线核对上游提交与 URDF 哈希，独立 URDF/MuJoCo 的 68 组 FK 再次通过；规划场景的七主关节原点、轴向、范围和零参考保持一致。附加夹爪仍使用装配 Demo 既有的固定夹持角，详见 [夹持约定](planning_scene_audit.md)。重建后 3 项 Demo 测试及完整真实键盘回归通过，截图更新为启动即七关节全零；R 明确标为装配预设。

同日移除 Demo 规划入口的前平面限制，记录见 [前平面限制移除验证](evidence/keyboard_no_front_plane.json)：23 项 CTest 和完整真实键盘回归通过。Baseline、RigidCache 均能对越过原前平面的核心位姿继续规划并展示结果；窗口测试中的该位姿因起点碰撞失败，保留请求路径与起点虚影，第一次回车及预览期间实体保持不动，失败结果不能执行。正常预设仍可在第二次回车后执行到完成。零关节点失败的窗口用例改用 `reject_blocked`；此前记录中的 `invalid_station_front_plane` 是修改前的历史结果。

此前的直接逆解与关节线性接近版本记录见 [接近段验证](evidence/keyboard_direct_preinsert.json)：当时 23 项 CTest 和真实键盘回归通过，名义装配预设可执行，但默认全零初值在 RigidCache 下出现 `ik_preinsert_no descending feasible step`。这份记录对应优化接近段之前的版本。

带碰撞检测的接近优化记录见 [优化接近段验证](evidence/keyboard_optimized_approach.json)：24 项 CTest 和完整真实键盘回归通过。原零位场景中，Baseline 的首个 IK 解本身存在自碰，RigidCache 的首个初值不收敛；新的筛选流程在两次 IK 尝试后为两个后端找到可用解，各生成 364 个轨迹点。窗口中完整规划实测约为 Baseline 1.50 s、RigidCache 0.74 s；第一次回车保持实体零位，第二次回车后的零位起步仿真执行到 `COMPLETE`。额外障碍物测试确认直连被阻挡、搜索得到绕行路径、捷径优化缩短关节路径，并通过 0.002 rad 步长且关闭缓存的复查；中断只返回起点侧已检查前缀。

最后 Q 阶段改为双向尽量旋转，记录见 [双向 Q 验证](evidence/keyboard_best_effort_q.json)：24 项 CTest 与完整真实键盘回归通过。名义场景中，Baseline、RigidCache 从七关节零位和装配预设起步，均生成 `0° → +90° → 0° → −90°` 的有效轨迹；RigidCache 的两次窗口执行均达到 `COMPLETE`，实际机构 Q 停在 −90°。控制器测试另检查执行经过正向峰值，并对两个后端的完整轨迹进行无缓存复查。已有碰撞回归场景停在 +63° 后转向，仍到达 −90°，验证“只能转一部分”不会使整条轨迹失效。窗口使用 llvmpipe，四次 NVIDIA 进程快照均没有测试 Demo 进程，测试窗口和 Xvfb 已退出。
