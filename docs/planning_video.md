# 三级兑换视频与更近的安装位置

按用户要求，机械臂基座到核心初始 E 点的**水平距离从 390 mm 缩短到 290 mm**。原机械臂基座坐标系保持不动，将 `Scenario::assembly_translation.x` 从 0.30 m 改为 0.20 m；所有物体尺度、夹爪—单元变换、地面高度、机构目标和碰撞阈值保持一致。这是实际相对位置变化。

直接沿用原 ready 分支会发生 j2/j4 自碰。离线使用固定随机种子 `20261011` 搜索更适合新安装位置的 ready 姿态，在候选 26 得到完整成功路径 `closer_x200_s26`。保留了 7 个直接平移探针、27 个起始姿态尝试，以及其中通过起点检查后的 12 次完整规划尝试记录。新演示不并入原先的 621 次冻结基准，也不代表覆盖范围或成功率得到提高。

| 当前视频 | 数值 |
| --- | ---: |
| 单次成功规划，给定已选定的 ready | 0.338193 s |
| 参数化轨迹时长 | 50.296019 s |
| 额外密采样碰撞复查 | 1526 点，通过，缓存关闭 |
| 独立 URDF FK 审计 | 1597 点，通过 |
| 最大独立位置误差 | 0.052216 mm |
| 最大独立轴角误差 | 0.000146578 rad |
| Q 连续对准时长 | 1.651790 s |

0.338193 s 包含本次规划的在线碰撞验证，**不包括上述离线起始姿态搜索**。视频按三次曲线 `3u²−2u³` 插值关节和机构状态，与规划验证一致；调用原 C++ `PlanningScene::set_state`，不以重新积分动力学代替已验证轨迹。每次渲染前再运行密采样 validator。它展示固定刚性携带下的三级轨迹回放，不是抓取或真实控制执行证明。

完整输入、成功和失败记录、生成探针与审计在 [证据目录](evidence/level3_video_closer/manifest.json)。该目录中的 `runs.csv` 保留全部 12 次规划，15 个未通过 ready IK/起点门禁的尝试另列于 `placement_scan.csv`。

## 重建视频

先按 [规划报告](planning_results.md) 构建 `build-planning`。视频额外需要 `libosmesa6-dev` 与 `ffmpeg`；不需要 Python MuJoCo 包。以下命令使用已归档的真实轨迹，输出应使用新路径：

```bash
python scripts/render_planning_video.py docs/evidence/level3_video_closer \
  --case closer_x200_s26 \
  --output artifacts/level3_planning/videos/level3-closer-10cm-replay.mp4
```

默认 1600×900、24 fps、1 倍速，包含全臂和夹爪/核心特写，最后保留 2 秒终态。以 `--still 25 --output artifacts/level3_planning/videos/closer-25s.png` 可输出指定轨迹时间的一帧；`--speed 4` 可生成 4 倍速版本。

脚本强制 `CUDA_VISIBLE_DEVICES=''`、Mesa llvmpipe 和 4 个 CPU 渲染线程，并核验实际 OpenGL renderer；ffmpeg 编码使用 2 个 CPU 线程。RGB 直接通过管道交给编码器，不落盘保存帧序列。临时编译文件在结束或异常时清理；日志 1 MiB 轮转、最多两份备份。最终视频先写临时文件，经 ffprobe 验证完整帧数后原子改名，同时输出 SHA-256、实际 renderer、模型/轨迹哈希和时长 JSON。

本次成品保留于 `artifacts/level3_planning/videos/level3-closer-10cm.mp4`。原始 390 mm 名义视频也保留，供比较，但交付以更近的 290 mm 版本为准。
