# 人工与数值验证记录

网站：https://viewer.robotsfan.com/

检查方式：加载本地完整目录，以网页关节滑块逐项进行正向和反向试转，再点击「重置」；逐项查看两种方向截图和关节连接，复核回零与初始 CAD 保存姿态一致。不是仅修改数值输入框。

- 检查 19 个 URDF 可动关节；全部回到零位，转动滑块中的 `5e-16` 为可忽略浮点误差。
- `zero.jpg`：关节面板保留的零位截图；`zero_full.jpg`：完整模型零位截图。
- `<joint>_plus.jpg` / `<joint>_minus.jpg`：该关节的正、反方向试转。
- `sheet_*.jpg`：上述原始截图组合，便于对照。
- `ui_joint_checks.json`：实际 UI 读数。
- `numerical_checks.json`：从 URDF 独立计算零位变换，再与 CAD 几何对比；含最终 URDF/STL 的 SHA-256。

本次验证针对导出主关节和夹爪坐标的局部运动与零位，不验证真实行程、碰撞安全、闭环夹爪内部传动或物理参数。
