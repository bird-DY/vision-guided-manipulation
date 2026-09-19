# vision_guided_manipulation

基于 ROS 2 的视觉引导移动操作项目。项目源于 LeoRobot 开源仿真和中控杯机械臂、灵巧手、Gemini335 现场联调，目标是建立可取消、可恢复、可回放和可量化评测的移动操作系统。

## 当前状态

已经完成：

- 导入并保留 LeoRobot 上游基线。
- 修正 Cartographer 资源安装目录。
- 建立 `zzx_interfaces`，包含目标、状态、任务消息以及视觉、底盘、机械臂、灵巧手和任务 Action。
- 在 WSL Ubuntu 22.04 / ROS 2 Humble 中完成接口包构建、自省和测试。
- 安装并核验 MoveIt 2、Nav2、ros2_control、Cartographer 与 Gazebo Classic 基线环境。

正在规划和实施：C++ 执行监控、可靠 RGB-D 定位、手眼/TCP 标定、MoveIt Task Constructor 抓放、行为树、比赛 HTTP 适配、故障注入和自动评测。路线图中的指标均为待实验的验收目标，不是现有成绩。

## 文档入口

- [企业级改进总体方案](docs/industrial_upgrade_plan.md)
- [实施任务与验收清单](docs/implementation_backlog.md)
- [统一 ROS 2 接口说明](ros2_ws/src/zzx_interfaces/README.md)

## 目录

```text
ros2_ws/src/Zzxrobot/       LeoRobot 上游代码与可复现基线
ros2_ws/src/zzx_interfaces  项目统一消息、服务和动作接口
configs/                    后续 robot/site profile
docs/                       架构、调研、实施和实验文档
scripts/                    后续环境、自检、记录与发布工具
tests/                      后续跨包契约和端到端测试
```

## 当前构建

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select zzx_interfaces
source install/setup.bash
colcon test --packages-select zzx_interfaces
colcon test-result --test-result-base build/zzx_interfaces/test_results --verbose
```

`zzx_interfaces` 已通过包级测试。上游 `mybot_description` 的空字节测试文件及 `mybot_cartographer` 的历史格式问题仍属于实施清单 B02，不能据此宣称全工作空间测试已经通过。

## 开发约定

- Python 用于视觉、训练、标定和离线分析；C++ 用于执行监控、规划、底盘对准和任务运行时。
- 通用数据优先使用 ROS 2 标准消息；长时间运动使用 Action。
- 任务成功必须来自执行反馈和物理结果验证，不能用固定 `sleep` 推断。
- 仿真与真机分别报告，未实测能力必须标为规划项。
- 提交描述使用中文，并写清改动后的可观察行为。

上游 LeoRobot 代码采用 MIT License，二次开发必须保留原版权和许可。第三方模型、SDK 和依赖按各自许可证单独核验。
