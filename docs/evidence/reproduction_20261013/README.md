# 本机复现证据

对应[本机复现报告](../../reproduction_20261013.md)，上级入口为[证据索引](../README.md)。复现起点为 `0fbd7f7`；规划算法与正式预算未改动，测试和打包修正见报告末节。

- `summary.json`：运行环境、测试、各协议统计、CAD 比较及媒体索引。
- `planner_success100/`：本次两后端各 100 个请求的完整逐例结果。
- `planner_quality/`：本次两后端各 308 个请求的完整逐例结果。
- `q_collision_recovery/`：本次两后端各 109 个请求的完整逐例结果。
- `planner_quality_historical/`：`e94db8e` 中修复 Q 之前的质量评测重跑；算法对应 `8859a1c`。
- 各 `audit_summary.json`：成功轨迹独立 URDF 审计结果及逐例证书。失败前缀只检查适用字段，不是可执行轨迹。
- `archive-audit-index.json`：1,971 条历史归档记录的完整校验索引。
- `historical-result-differences.json`：新旧成功请求集合比较；旧严格 Q 少通过的六例均为时间预算耗尽。
- `final-tests-build-ctest.log`、`final-tests-build-headless-ctest.log`、`delivery-verify-ctest.log`：窗口版、无窗口版和初版交付包重新构建的测试结果。
- [最终包校验](tech-core-reproduction-final.zip.verification.json)、`final-delivery-execution.json` 和 `final-delivery-ctest.xml`：最终归档包的校验、独立解压构建命令及 25 项测试结果。包的源提交以校验文件为准，不代表后续文档提交。
- `media-validation.json`：8 段归档轨迹回放视频的元数据与检查结果；视频本体保存在本机。
- `versions.json`、`reference-checksums.json`、`core-cad-comparison.json`：工具版本、61 个参考文件校验和官方核心坐标比较。

全部原始轨迹、二进制、源代码快照、命令、失败现场、视频与大型 CAD 输入保存在本机 `artifacts/reproduction-20261013/` 及 `build/fidelity-research/official/`。本目录保存精简证据，不重复提交大文件。
