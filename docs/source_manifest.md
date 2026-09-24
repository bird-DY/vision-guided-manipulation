# 上游来源与可追溯清单

## LeoRobot 基线

- 导入文件：`LeoRobot-master.zip`
- 原始位置：Windows `D:/EdgeDownload/LeoRobot-master.zip`
- SHA-256：`3b40a96683c74fa92ca9dae0d63d95592abba3ce64c31f2c158e7cf8afcfef2c`
- 首次导入提交：`5f1aae4 chore: import upstream robot project baseline`
- 仓库内目录：`ros2_ws/src/Zzxrobot`
- 上游许可：仓库所附 `LICENSE` 为 MIT；第三方模型和依赖不自动继承该许可。

原压缩包没有保留可验证的 Git remote 与上游 commit，因此不能声称对应某个公开仓库的精确版本。后续如找到官方仓库，应记录 URL、commit 和导入差异，但不能覆盖本条哈希证据。

## 本地比赛实现

中控杯决赛代码位于独立目录 `/home/zzx/zhongkong_final`，用于需求、协议和故障案例参考，不作为本仓库的隐式依赖。需要复用的功能必须通过明确接口迁移，并在提交中保留来源说明。

## ROS 2 依赖来源

ROS 2、MoveIt 2、Nav2、ros2_control、Gazebo ROS 和 Cartographer ROS 通过 Ubuntu 22.04 的 ROS 2 Humble APT 仓库安装。当前基线不使用未声明的源码 overlay；具体版本见 `docs/environment.md` 和每次发布保存的 `dpkg-query` 清单。

## Python 视觉环境

`requirements/vision-cu128.lock.txt` 来自当前项目虚拟环境的 `pip freeze --local`。ROS Python 包来自系统 APT，并通过 `--system-site-packages` 暴露给虚拟环境，因此不会出现在该锁文件中。

权重文件、数据集和标定数据不得只记录文件名。后续数据清单至少保存 SHA-256、来源、许可、采集配置、用途划分和隐私级别。
