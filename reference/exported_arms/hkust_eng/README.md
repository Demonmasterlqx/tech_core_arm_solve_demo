# HKUST ENG Open

主文件：`hkust_eng.urdf`。网格：`meshes/*.stl`，二进制 STL，单位为米。整个文件夹可独立移动，URDF 使用相对路径。

## 在查看器中打开

打开 https://viewer.robotsfan.com/ ，点击「加载文件夹」，选择本目录。也可把 URDF 与整个 meshes 文件夹一同拖入。坐标系选择 `+Z`；点击「重置」返回 CAD 零位。

## 模型与零点

- 作者：香港科技大学 ENTERPRIZE。
- 来源装配：ENG_full_3.24/Arm_newopen.SLDASM；底座取自 ENG_Baseopen 的实际可见版本。
- 关节结构：6 个主轴 + 1 个夹爪开合自由度（2 个联动夹爪关节）。URDF 共 8 个可动关节、9 个 link。
- **所有关节 q=0 对应原 SolidWorks 独立臂装配体保存姿态**，按用户确认采用。没有为外观对称而改变零位；这不表示实机编码器已经标定。
- 每个 link 的局部原点位于对应关节，初始方向统一；关节 `axis` 可以不是单一 X/Y/Z 方向。转动角用弧度，平移用米，遵循右手定则。
- `joints.csv` 列出关节父子关系、原点、轴向及夹爪 mimic 关系。`manifest.json` 记录零件归属、CAD 配合证据和坐标变换。

## 验证

已在指定网站实际加载，逐个操作所有导出关节正向、反向并重置，检查关节附近连接与回零姿态。记录与截图位于 `verification/`。

URDF 单根无环、全部 STL 路径、单位轴向、配合轴同轴性以及零位几何重建通过检查。保留几何的最大单坐标误差为 1.49e-08 m（约 1.49e-05 mm），来自 STL 单精度存储。完整结果及文件校验和见 `verification/numerical_checks.json`。

## 范围说明

差动腕部使用独立的输出俯仰/旋转关节；电机转子和差动传动未另建关节。相机与 UWB 辅助结构保持保存姿态。整机中隐藏的旧 yaw 底座和旧电机已换成实际使用的版本。

关节上下限只是查看器显示范围，未表示真实机械行程。按最终要求，未导出/验证质量、惯量或驱动参数；`effort`、`velocity` 为 0 占位值。collision 复用可视 STL。本包用于几何与运动学查看。

## 源数据完整性

源中可用的机械臂几何已覆盖；没有未导出的几何引用。

## 来源和使用条款

原始开源帖：https://bbs.robomaster.com/article/1947243?source=4。

香港科技大学 ENTERPRIZE 的原创开源内容采用 CC BY-NC-SA 4.0；本包是其几何转换衍生文件，保留署名、非商业和相同方式共享要求。第三方零件遵循其原有条款。许可：https://creativecommons.org/licenses/by-nc-sa/4.0/ 。
