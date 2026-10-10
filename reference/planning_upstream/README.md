# RM26 工程机器人上位机开源仓库

这个仓库用于整理和发布 PnX RM26 工程机器人上位机控制框架的开源资料，包含源码快照、开源报告和验证记录。

本项目面向已经具备一定 ROS 2 基础的队内或社区开发者。读者可以通过本仓库了解 RM26 工程机器人上位机控制框架的整体设计、部署方式、CAN 电机设备接入方式、ros2_control hardware interface/controller 编写方法，以及本赛季机械臂控制和底盘功率控制的实现思路。

## 仓库内容

| 目录 | 内容 |
| --- | --- |
| `src/` | 整理后的开源源码，包含 `RMHardwearInterface`、`RMController`、`RMManager` 以及通信适配相关代码。 |
| `report/` | 面向读者的多文件 Markdown 开源报告，是理解和使用本仓库的主要入口。 |
| `review/` | 文档审阅、远端验证、飞书同步等过程记录，用于追溯报告编写和验证过程。 |

## 快速入口

- [开源报告总览](report/README.md)
- [整体框架介绍](report/01_overall_architecture.md)
- [部署、构建与启动](report/02_deploy_build_run.md)
- [CAN 通信与 Sland CAN](report/03_can_and_sland_can.md)
- [整车 control.launch.py 数据流](report/08_control_launch_data_flow.md)
- [未来展望与致谢](report/11_future_outlook_and_acknowledgements.md)

## 技术范围

本仓库主要覆盖以下内容：

- Ubuntu 24.04 + ROS 2 Jazzy 下的工程机器人上位机控制框架。
- 基于 ros2_control 的 `hardware_interface`、controller 插件和 `controller_manager` 启动链路。
- DJI/DM 电机等 CAN 设备的上位机接入与抽象。
- Sland CAN、SocketCAN、vcan 等调试和运行路径。
- RM26 version3 整车控制 launch 的数据流。
- 机械臂控制、底盘功率控制、裁判系统/图传通信相关说明。

## 推荐阅读方式

第一次阅读建议从 [report/README.md](report/README.md) 开始，按章节顺序阅读。部署和二次开发时，优先参考：

1. [部署、构建与启动](report/02_deploy_build_run.md)
2. [如何编写 hardware interface](report/04_write_hardware_interface.md)
3. [基于 CANDeviceInterface 开发 CAN 设备](report/05_can_device_interface_development.md)
4. [如何编写 controller](report/07_write_controller.md)

如果只想快速理解现有整车如何跑起来，可以直接阅读 [整车 control.launch.py 数据流](report/08_control_launch_data_flow.md)。
