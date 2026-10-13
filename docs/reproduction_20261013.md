# 2026-10-13 本机复现记录

[文档索引](README.md) · [本次证据](evidence/reproduction_20261013/README.md) · [原 100 请求报告](planner_success100.md)

本次从 `0fbd7f7` 开始，在 Ubuntu 24.04 / WSL2 上重新安装依赖、编译、运行和审计。规划算法、模型、碰撞阈值及正式评测的时间预算没有修改。原始输出集中于 `artifacts/reproduction-20261013/`；每组命令、退出码和耗时在该目录的 `plans/`、`*-execution.json` 和 `logs/` 中。机器可读汇总为该目录的 `summary.json`。

本文将重新规划、历史程序重跑、归档恢复与视频回放分别记录。失败请求保留在各自分母中；失败轨迹前缀不视为可执行轨迹。历史代码在独立副本中构建，主工作区不切换历史版本。

## 构建与功能

| 项目 | 本次结果 |
| --- | --- |
| 完整窗口 Release 构建与 CTest | **25/25 通过** |
| 无窗口 Release 构建与 CTest | **23/23 通过** |
| 交付 ZIP 在新路径解压、重新构建与 CTest | **25/25 通过** |
| 核心一级至四级演示 | 全部 SUCCESS，保存实际状态 CSV |
| Baseline / RigidCache 库调用示例 | 均成功，并通过无缓存复验 |
| 独立机械臂 X11/XTest | 方向、关节模式、停止、暂停、复位与截图通过 |
| 联合规划 Demo X11/XTest | 两后端、成功/失败预览、两次回车、归零、场景编辑与执行终态通过 |
| 三个窗口入口与截图 | 使用 llvmpipe 软件渲染通过 |

最终测试日志为 `logs/final-tests-build-ctest.log`、`logs/final-tests-build-headless-ctest.log` 和 `logs/delivery-verify-ctest.log`。首次运行中 `planning_approach` 的失败日志也保留，原因及修改见后文。

可直接启动联合 Demo：

```bash
./scripts/run_planning_demo.sh
```

需要可见的桌面显示才能手动操作；无显示时脚本使用不可见的 Xvfb。首次 Enter 规划和预览，成功后第二次 Enter 执行；R 载入装配预设，Backspace 回到七关节零位。

## 当前代码的固定输入评测

### 两后端各 100 请求

沿用 `docs/evidence/planner_success100/cases.csv`，每次 5 s，两个后端各包含 50 个零位起点和 50 个预设起点。

| 指标 | Baseline | RigidCache |
| --- | ---: | ---: |
| 有效轨迹 | **31/100** | **31/100** |
| 完整 Q 双向 ±90° | **14/100** | **15/100** |
| 历史记录的完整 Q | 19/100 | 18/100 |
| 本次最长在线耗时 | 4.689 s | 4.665 s |

200 条记录全部通过独立 URDF 审计，62 条成功轨迹均通过 C++ 无缓存验收。各后端的成功请求集合也分别与其历史结果一致；两后端成功集合的并集为 32 个。固定墙钟预算下，CPU 性能、负载和数值实现会影响候选选择、重试及可完成的 Q 行程；本次没有延长正式预算来追平旧数字。

结果在 `planner_success100/{baseline,rigid_cache}/results/`，独立审计在 `planner_success100/audit/`。

### 扩大范围质量评测与 Q 换解

使用仓库现有全部固定请求，未重新采样或筛选可达请求。

| 请求集 | Baseline | RigidCache |
| --- | ---: | ---: |
| 质量评测开发组 | 19/64 | 19/64 |
| 质量评测扩展组 | 62/200 | 62/200 |
| 质量评测旧 40 位姿组 | 37/40 | 37/40 |
| 质量评测阻挡/远距离负例 | 正确拒绝 4/4 | 正确拒绝 4/4 |
| Q 换解开发组，含截图请求 | 20/65 | 20/65 |
| Q 换解旧 40 位姿组 | 37/40 | 37/40 |
| Q 换解负例 | 正确拒绝 4/4 | 正确拒绝 4/4 |

质量评测共 616 条、Q 换解评测共 218 条记录，独立审计均为零错误。前者有 236 条成功轨迹，后者有 114 条。开发组成功数比历史报告少一个请求，已如实保留。结果分别在 `planner_quality/` 和 `q_collision_recovery/`。

## 历史版本与归档

| 内容 | 版本 / 方式 | 本次结果 |
| --- | --- | --- |
| 九方法 621 次评测 | `afc7f85` 原版本及原包装器 | 621/621 请求完整，每种方法 30/69 成功，与历史成功数量一致；全部独立审计通过 |
| 离线起点搜索的 40 位姿 | `880a5eb` | 21/40 成功，与历史记录一致；独立审计通过 |
| 固定起点、严格 Q 的在线重试 | `d458496`，3 s | 14/40 成功；比历史少的 6 个成功请求均因 `online_deadline` 失败。本次完整运行且审计通过 |
| 旧 best-effort Q | `b4472ed`，与归档源码逐文件核对 | 第 37 个位姿触发在线/无缓存不一致后终止，仅写出前 36 个请求；不作为完整评测 |
| 同一 40 位姿的当前代码 | 当前规划库，3 s | **37/40** 成功，其中完整 Q 35 个、部分 Q 2 个；完整无缓存及独立审计通过 |
| 三批归档及其中的对照试验 | 恢复压缩轨迹并逐项校验 SHA-256 | **1,971 条记录全部审计通过**，其中 281 条成功轨迹；其余只验失败前缀的适用字段 |

旧 Q 程序的失败是 `pose37 online/uncached validation mismatch`，涉及夹爪与携带单元的固定配对，返回距离约 −0.558 mm。其源码与 `docs/evidence/pose40_q_range/source/` 一致。当前仓库已包含后续刚性配对距离修复，使用当前版本可以完成同一协议；本次没有修改历史实现、放宽碰撞阈值或跳过失败位姿。

旧九方法包装器不接受新版 `--motion-speed-scale` 参数，因此使用该提交自带的包装器重跑。最初的启动失败输出保留在 `methods-621-historical/`，正式完整结果在 `methods-621-frozen/`。

归档结果位于 `restored/`；`archive-audit-index.json` 列出全部主试验和控制组。先前版本的时间数字不应与本机结果直接混用。

另在 `e94db8e` 独立副本中重跑了 Q 局部换解修复前的完整质量评测（其规划算法对应 `8859a1c`）。两个后端各 308 个请求，616 条记录独立审计全部通过；原始结果在 `planner_quality_historical/`，配对汇总在 `planner_quality/paired-before-q/`。两版本各后端的扩展组均为 62/200 有效轨迹；完整 Q 的 Baseline / RigidCache 数量由 12/11 增至当前的 30/32，旧 40 位姿组则由各 14 增至各 36。这里比较固定输入的实际结果，不将本机耗时与原机器的耗时直接作性能结论。

## 资产、CAD 与文档

- 61 个规则、机械臂和上游参考文件通过已有 SHA-256 清单校验。
- 从官方公开接口重新取得 V2.0.0 手册并校验 MD5；与仓库 V2.2.0 对应页面提取的 18 个几何插图逐字节一致，18/18 个图像哈希也与历史记录一致。
- 使用 **Blender 4.5.0** 重导出核心，60/60 个 OBJ 与仓库文件逐字节一致。
- 重新运行核心、机械臂和联合场景生成器；运行模型不变。重新生成夹爪分解后，仅审计中的运行耗时变化。
- 独立 URDF 与 MuJoCo FK 比较 68 个姿态、14 个 body 加工具帧；最大位置元素误差 `5.55e-16 m`，旋转矩阵元素误差 `1.44e-15`。
- 官方能量单元和 1.25 GB 整场 STEP 均通过大小、MD5 和 SHA-256 校验。整场读取及导出完成，记录 466 个装配树条目。
- 核心 CAD 提取结果为 418 个部件、4,892 个面、1,673 个壳、0 个闭合实体，与历史报告一致。E 点完全相同，坐标变换最大元素差 `2.28e-13`。
- 能量单元导入、孔修订与 **6/6 离线测试**通过。少量共面三角剖分不同；原始网格顶点与法线一致，修订件最大顶点差约 `1e-10 m`，最大总表面积差 `9.51e-14 m²`。未覆盖仓库中的运行模型。
- 三份手册的 PDF 和离线 MathML HTML 已生成，中文与公式预览检查通过，PDF 文本可提取。

资产对照与几何结果在 `models/`；主要版本记录在 `versions.json`。运行使用 MuJoCo 3.3.7、GCC 13.3、Python 3.12.3、NumPy 1.26.4；软件渲染为 Mesa llvmpipe / LLVM 20.1.2。

## 输出与复现命令

```bash
# 当前功能与两种构建
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
CUDA_VISIBLE_DEVICES='' ctest --test-dir build --output-on-failure
cmake -S . -B build-headless -DCMAKE_BUILD_TYPE=Release -DTECH_CORE_VIEWER=OFF
cmake --build build-headless -j4
CUDA_VISIBLE_DEVICES='' ctest --test-dir build-headless --output-on-failure

# 固定 100 请求：输出路径必须为新目录
python3 scripts/run_planner_quality.py --suite docs/evidence/planner_success100 \
  --output artifacts/new-100/baseline --split all --method baseline --cpu 0 --timeout 5
python3 scripts/run_planner_quality.py --suite docs/evidence/planner_success100 \
  --output artifacts/new-100/rigid_cache --split all --method rigid_cache --cpu 2 --timeout 5
OPENBLAS_NUM_THREADS=1 python3 scripts/summarize_planner_quality.py \
  --suite docs/evidence/planner_success100 \
  --runs artifacts/new-100/baseline artifacts/new-100/rigid_cache \
  --certificate-root artifacts/new-100 --output artifacts/new-100/audit
```

CPU 编号须在当前进程允许的集合中；本次使用 0/2 分别运行两后端。当前机器的全部具体命令另存于 `artifacts/reproduction-20261013/plans/`，包含历史副本路径和视频参数。正式规划、无缓存复验与独立审计的时间分别记录。

基础系统依赖见 README。完整离线流程还使用 `libosmesa6-dev`、`ffmpeg`、`python3-venv`、`python3-numpy`、`python3-matplotlib`、`python3-pil`、`pandoc`、`texlive-xetex`、`texlive-latex-extra`、`texlive-lang-chinese`、`texlive-fonts-recommended`、`lmodern`、`fonts-noto-cjk`、`fonts-dejavu-core`、`fonts-urw-base35`、`poppler-utils` 和 `unzip`。CAD 环境安装 `cadquery-ocp==7.8.1.1`，完整离线包版本见 `versions.json`。

初版交付包为 `artifacts/reproduction-20261013/tech-core-reproduced.zip`，配套 SHA-256、文件清单、ZIP 校验和新路径重建记录在相邻的 `.sha256`、`.report.json`、`.verification.json` 中。包内为独立 Git 仓库、MuJoCo 依赖和三份手册；大体积实验轨迹、CAD 输入及视频保存在本机 `artifacts/`，不重复放入源码交付包。

## 完整视频与媒体验证

8/8 段视频已生成。汇总为 1920×1080 / 20 fps，特写为 1600×900 / 24 fps；保持各归档轨迹的真实运动时长及原始播放速度。每次渲染前重新密采样验收，生产脚本检查实际 CPU 渲染器、ffprobe 帧数及文件 SHA-256；另提取实际视频帧供检查。

这些视频回放归档轨迹；本轮新规划的统计与原始 CSV 单独保留。

| 视频 | 时长 / s | 完整帧数 |
| --- | ---: | ---: |
| [离线 40 位姿汇总](../artifacts/reproduction-20261013/media/pose40_q_sweep.mp4) | 132.700 | 2654 |
| [离线位姿特写](../artifacts/reproduction-20261013/media/pose40_q_sweep-detail.mp4) | 25.792 | 619 |
| [严格 Q 在线 40 位姿汇总](../artifacts/reproduction-20261013/media/pose40_online.mp4) | 138.150 | 2763 |
| [严格 Q 在线位姿特写](../artifacts/reproduction-20261013/media/pose40_online-detail.mp4) | 25.792 | 619 |
| [尽量旋转 Q 的 40 位姿汇总](../artifacts/reproduction-20261013/media/pose40_q_range.mp4) | 137.900 | 2758 |
| [Q 范围与安全停止特写](../artifacts/reproduction-20261013/media/pose40_q_range-detail.mp4) | 24.750 | 594 |
| [安装间距缩短 100 mm](../artifacts/reproduction-20261013/media/level3_video_closer.mp4) | 52.333 | 1256 |
| [降低基座、倾斜核心、4 倍运动速度](../artifacts/reproduction-20261013/media/level3_video_lowered_tilted_4x.mp4) | 14.458 | 347 |

媒体元数据和预览索引见 `artifacts/reproduction-20261013/media-validation.json`。

精简实测记录另保存于 [本次证据目录](evidence/reproduction_20261013/README.md)。

## 本次可复现性修正

1. 恢复解压时丢失的七个脚本执行权限，并忽略本机 `.env`。
2. 无缓存绕障功能测试原先给出 3 s 搜索预算，在本机稳定超时。改为 10 s，CTest 外层上限改为 60 s；原有超时和取消断言仍保留。**正式规划和评测预算未修改。**
3. 质量汇总脚本的 `--original` 改为可选，以便仅凭本轮新结果完成独立审计；提供旧结果时仍执行配对比较。
4. 文档打包预检补充 `lmodern.sty` 和 `pzdr.tfm`，在缺少 PDF 字体依赖时提前给出具体缺项。

最终补入本次报告和精简证据的交付包为 `artifacts/reproduction-20261013/tech-core-reproduction-final.zip`。其运行源码与已完成新路径构建测试的初包一致；[最终包校验记录](evidence/reproduction_20261013/tech-core-reproduction-final.zip.verification.json)确认独立解压复核通过、Git 工作区干净及 25/25 项 CTest 通过。该文件记录的是当时交付包的源提交和 SHA-256，后续文档整理不会改变该归档包，也不将它视为当前 HEAD 的新打包结果。
