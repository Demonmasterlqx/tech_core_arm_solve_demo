# 三维模型来源与版本核对

核对日期：2026-10-11。科技核心使用用户指定的 HKUST Blender 模型；能量单元使用官方 STEP 外形及规则孔接口。官方整场 CAD 用于独立尺寸比较，不作为运行时依赖。用户已接受小幅尺寸差异。

## HKUST 科技核心模型

原始文件来自相邻仓库 `tech_core_hkust/src/blender/exchange.blend`，上游为 [HKUST ENTERPRIZE](https://github.com/hkustenterprize/RM2026-Engineer-Assembly-Algorithm)。本仓库保留原文件于 `reference/hkust/`，附 MIT 许可和 SHA256 `9490131f84030944de45f14fc5dd682fe1ae2cac779d211ac421b9a412093e9a`。它是队伍依据手册重建的模型，不标为官方 CAD。

导出采用 Blender 4.5.0 的求值网格，应用原有修改器，保留全部 59 个对象、60 个材质分件、34,792 个三角面和逐角法线，不简化或缩放。坐标换算为 `[X,Y,Z]=[Blender Z−0.112000003457, Blender X, Blender Y]`，单位米，旋转行列式为 +1，E 点来自原场景明确的定位空物体。正面观察时 +Y 朝画面右侧。模型中电磁铁细节超出名义 E 约 4.627 mm，原样保留。

```bash
blender --background --factory-startup --threads 4 \
  --python scripts/cad/export_hkust_blend.py
python3 scripts/generate_model.py
```

导出清单 `models/meshes/core/manifest.json` 包含原对象、材质、修改器、边界、坐标矩阵及每个 OBJ 的 SHA256。正常 C++ 构建/运行只需要已入库的 XML 和 OBJ，不依赖 Blender、Python、原相邻目录或大体积场地 STEP。详细 MARK 尺寸、外壳差异见 [几何审计](fidelity_geometry_audit.md)。

## 官方 CAD

官方来源：[【官方开源】RMU 2026 场地图纸&可移动道具开源](https://bbs.robomaster.com/article/814728)。作者为 `RoboMaster 高校赛务`（authorId 39696），发布于 2025-10-22，更新于 2026-06-30。此作者也是[规则/资料中心](https://bbs.robomaster.com/wiki/20204847)的维护者。公开资料接口为 `POST https://bbs.robomaster.com/developers-server/rest/posts/info/814728`，空 JSON 请求体即可读取。

| 文件 | 官方标明适用版本 | 字节数 | 官方 MD5 | 下载后 SHA-256 |
| --- | --- | ---: | --- | --- |
| RMUC2026_V2.0.0.stp | RMUC 2026 规则 V2.0.0 | 1,254,821,405 | `cb3da25791d000eca9c0e9bcd259f5b1` | `8dfe9ebd761e44d91361b3e593bc05416329112217b58cb35800b3cde2ffae33` |
| 能量单元.stp | RMUC & RMUL 2026 规则 V1.0.0 | 842,155 | `07a99a364d64ffe056a6a29f1b6450ce` | `35d105a221f4527e5c5e84ac607c94c797ec2e19c734974de4c5485c2c2d5ee7` |

- [场地 STEP 原始下载](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/f637e13eadfc4602a76c7a05c54f71271782803452461/RMUC2026_V2.0.0.stp)
- [能量单元 STEP 原始下载](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/21cbbecaaceb47729bc477cd12f8de5b1761107116144/%E8%83%BD%E9%87%8F%E5%8D%95%E5%85%83.stp)

下载文件分别保存为 `build/fidelity-research/official/RMUC2026_V2.0.0.stp` 和 `build/fidelity-research/official/energy_unit.stp`。两者均已核对字节数和官方 MD5，并计算上述 SHA-256。场地文件使用 `.part` 续传，完成校验后原子改名。原始 API 响应、下载清单与终态校验在同目录的 `post_814728.json`、`official_files.json` 和 `field_download_verified.json`。原始场地文件约 1.25 GB，作为可重新下载的构建输入保存，不应将整份场地重复嵌入仿真运行资产。

官方原文同时声明：

> 本文件为 RMUC 2026 的场地模型，仅做参考使用，若此模型与规则手册标注或与比赛现场场地有结构出入，以规则手册和现场场地为准。

因此，导入官方 STEP 可以证明与官方发布几何的比例和结构一致，但仍不能凭此声称已对比赛实物完成制造公差或材料光学性质验证。

## V2.0.0 CAD 与用户指定 V2.2.0 规则的关系

用户指定的本地手册为 V2.2.0（2026-08-07）。[官方规则页面](https://bbs.robomaster.com/wiki/20204847/809871)列出的该文件为 21,018,604 bytes，MD5 为 `d4808d8f91575d89e2fe4f7ba2f4fb3f`。

另下载官方 V2.0.0（2026-06-26）PDF，21,012,597 bytes，MD5 为 `505655c0c3bbe7cd92f7ace5fb28376f`，作为 CAD 版本对应的核对基准。未下载所有旧场地 CAD。

对 V2.0.0 的 PDF 阅读器第 80–83 页，与 V2.2.0 的第 81–84 页逐一提取原始嵌入图像；这些页覆盖图 5-5 至图 5-10，包含科技核心装配机构、P/Q 轴几何、MARK 灯尺寸、面板尺寸和装配坐标系。提取出的 **18 个图像/透明度层逐字节一致**。其中七个主体图像（其余为透明度层和页眉标识）尺寸分别为 961×641、1766×994、1145×935、1913×1869、736×758、810×882 和 843×826。已入库证据为 `docs/evidence/geometry_image_comparison.json`。这证明所核对的几何/标识插图在两版本之间未改变。

§5.3.3 正文比对得到的相关变化包括：

- 步骤 3 将移动阻力的方向明确为沿 z 轴正向。
- 步骤 5 从“正确位置至多 0.2 s 后灯效提示”，改为“正确位置至少 0.3 s、至多 0.5 s 后选手端提示”；图 5-15 的标题也调整为灯效示意。
- Q 轴回零力矩上限由 0.6 N·m 调整为 0.3 N·m；正确位置停留超过 2 s 后的保持方式明确为约 0.5 N·m 主动力矩。删除旧的 0.3–0.6 N·m 旋转阻力表述。

仿真的时序与力矩逻辑必须继续以 V2.2.0 为准，不能随 CAD 版本退回 V2.0.0 的机制。几何插图一致并不证明所有实物零件均未改动；导入后仍需核对原始 CAD 尺寸和可见结构。

## 未用作官方几何依据的社区模型

检索还找到[南京理工大学 Alliance 的五自由度科技核心开源](https://bbs.robomaster.com/article/1938962)。它明确是训练用复刻：大臂角度 9° 一档、三台 MG-8010E-i36 电机、打印件固定灯带。即使外观接近，也不能替代官方 1:1 几何，因此未下载或纳入模型。
