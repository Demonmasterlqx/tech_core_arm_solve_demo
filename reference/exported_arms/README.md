# 四套机械臂 URDF + STL

每套一个独立文件夹，均已在 https://viewer.robotsfan.com/ 实际加载并逐关节试转、回零检查。零点采用用户确认的 SolidWorks 独立臂装配体保存姿态；物理参数不在最终要求内。

| 机器人 | URDF | Link/STL 数 | 可动关节数（含 mimic） | 源空几何引用 |
|---|---|---:|---:|---:|
| [HKUST ENG Open](hkust_eng/README.md) | [hkust_eng.urdf](hkust_eng/hkust_eng.urdf) | 9 | 8 | 0 |
| [首都师范大学 PIE RM2026 工程机器人](pie_eng/README.md) | [pie_eng.urdf](pie_eng/pie_eng.urdf) | 9 | 8 | 7 |
| [追梦 RM2026 国赛单臂工程](zhuimeng_single/README.md) | [zhuimeng_single.urdf](zhuimeng_single/zhuimeng_single.urdf) | 9 | 8 | 0 |
| [追梦 RM2026 国赛双臂工程](zhuimeng_dual/README.md) | [zhuimeng_dual.urdf](zhuimeng_dual/zhuimeng_dual.urdf) | 20 | 19 | 0 |

共 47 个 STL、43 个可动关节。每个机器人目录提供中文说明、关节表、来源条款、零件清单和原始验证截图。PIE 的 7 个空引用来自上游源数据，详细名称列于其 README 和 manifest，未伪造替代件。

打开方式：查看器 → 加载文件夹 → 选择一个机器人目录 → 重置。URDF 的 mesh 路径为相对路径，移动文件夹时保留 meshes 子目录。

导出与验证代码位于 ../tools/arm_export，采用独立 Git 仓库管理。finished 文件在最终检查完成后生成。
