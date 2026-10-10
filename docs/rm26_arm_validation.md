# RM26 机械臂验证记录

验证日期：2026-10-11。C++17 / GCC 11.4、MuJoCo 3.3.7；软件渲染为 `llvmpipe (LLVM 15.0.7, 256 bits)`。以下结果来自实际编译、加载模型和运行，不是静态代码推断。结构化记录见 [runtime.json](evidence/rm26-arm-runtime.json)。

| 项目 | 实际结果 |
|---|---|
| 有图形 Release 构建 | 编译通过，CTest 17/17 |
| 全新无图形 Release 构建 | 编译通过，CTest 17/17 |
| 源 URDF 与 MJCF | 68 个姿态 × 14 个 body + tool；最大位置差 `4.44e-16 m`、旋转矩阵元素差 `1.33e-15` |
| 关节位置指令 | j0 目标 0.2 rad，2 s 后 `0.199999977369 rad` |
| 关节速度指令 | j0 0.2 rad/s，0.5 s 实际位移 0.1 rad |
| 末端速度指令 | +X 0.03 m/s，0.5 s 实际位移 `0.014996174080 m` |
| 实际 GLFW 键盘事件 | W/S/A/D/Q/E 和四个方向键均沿规定世界轴运动；每次松键后保持位置 |
| 模式切换与关节 jog | 第七关节位置模式、速度模式均实际运动，其余关节保持 |
| 暂停/恢复/复位 | 暂停时仿真时钟和关节冻结；恢复无残留运动；复位恢复 home |
| 截图 | 实际 PNG 已检查；截图前后关节位置、速度和末端位置一致 |
| 资源回收 | 验证结束无 rm26_arm 或 Xvfb 进程；临时验证程序和下载 `.part` 已清理 |

位置与朝向一致性表示转换正确，不是机械臂实物精度标定。所有关节测试运行在无力矩、无接触反作用的 mock 上。CTest 包含原项目 11 项和本次 6 项：控制契约、静止、六方向演示、Cartesian CLI、NaN 拒绝、冲突控制参数拒绝。控制契约覆盖七关节硬限位、限速、有限位置伺服、速度积分、停止、独立 URDF home/zero FK、六方向平移、姿态保持、奇异位形和非法输入。

复现构建测试：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure -j4
cmake -S . -B build-arm-headless -DCMAKE_BUILD_TYPE=Release -DTECH_CORE_VIEWER=OFF
cmake --build build-arm-headless -j4
ctest --test-dir build-arm-headless --output-on-failure -j4
python3 scripts/import_rm26_arm.py --verify
```

真实键盘测试通过 Python 标准库 ctypes 调用 X11/XTest，找到 GLFW 窗口、取得焦点、按下/释放按键，并断言输出的实际 MuJoCo FK 位移，无需 xdotool：

```bash
env CUDA_VISIBLE_DEVICES='' LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe \
  __GLX_VENDOR_LIBRARY_NAME=mesa LP_NUM_THREADS=4 \
  xvfb-run -a -s '-screen 0 1440x1000x24 -nolisten tcp' \
  python3 scripts/test_arm_keyboard.py
```

需要系统 `libX11.so.6`、`libXtst.so.6`、`xvfb`、`xauth`。脚本有事件等待超时，成功或失败都会终止自己的 viewer、释放按键和 X11 连接；Xvfb 由外层清理。日志与报告写入 `artifacts/rm26_arm/keyboard/`，不会无限增长。一次按键实际持续的仿真时间会受软件渲染帧率影响，因此用位移方向、停止和模式行为作为断言，不把墙钟按键时长当成精确仿真时间。

完整原始输出保留在本地 `artifacts/rm26_arm/`；随提交保存结构化结果、模型验证报告和一张截图。现有模型/实验数据未清理或覆盖。
