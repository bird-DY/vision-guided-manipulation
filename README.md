# vision_guided_manipulation

基于 ROS 2 的视觉引导移动操作项目。项目源于 LeoRobot 开源仿真和中控杯机械臂、灵巧手、Gemini335 现场联调，目标是建立可取消、可恢复、可回放和可量化评测的移动操作系统。

## 当前状态

已经完成：

- 导入并保留 LeoRobot 上游基线。
- 修正 Cartographer 资源安装目录。
- 建立 `zzx_interfaces`，包含目标、状态、任务消息以及视觉、底盘、机械臂、灵巧手和任务 Action。
- 在 WSL Ubuntu 22.04 / ROS 2 Humble 中完成接口包构建、自省和测试。
- 安装并核验 MoveIt 2、Nav2、ros2_control、Cartographer 与 Gazebo Classic 基线环境。
- 冻结 Humble 环境、Python 视觉依赖和上游压缩包来源，提供一键安装与环境自检脚本。
- 分离纯模型、Gazebo 控制和 MoveIt mock 控制，统一机械臂及夹爪控制器关节所有权。
- 修复上游 Python 空字节测试、Cartographer 启动格式和原型节点静态检查问题。
- 建立严格 robot profile、工具注册表和自动运行标定门禁，隔离仿真六轴与比赛七轴/O10 配置。

正在规划和实施：C++ 执行监控、可靠 RGB-D 定位、手眼/TCP 标定、MoveIt Task Constructor 抓放、行为树、比赛 HTTP 适配、故障注入和自动评测。路线图中的指标均为待实验的验收目标，不是现有成绩。

## 文档入口

- [企业级改进总体方案](docs/industrial_upgrade_plan.md)
- [实施任务与验收清单](docs/implementation_backlog.md)
- [开发环境基线](docs/environment.md)
- [上游来源清单](docs/source_manifest.md)
- [Robot profile 使用说明](docs/robot_profiles.md)
- [统一 ROS 2 接口说明](ros2_ws/src/zzx_interfaces/README.md)

## 目录

```text
ros2_ws/src/Zzxrobot/       LeoRobot 上游代码与可复现基线
ros2_ws/src/zzx_interfaces  项目统一消息、服务和动作接口
configs/                    Robot、工具及后续 site profile
docs/                       架构、调研、实施和实验文档
scripts/                    后续环境、自检、记录与发布工具
tests/                      后续跨包契约和端到端测试
requirements/               按硬件平台区分的 Python 版本锁
```

## 当前构建

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

完整环境安装、自检和全工作空间测试见 `docs/environment.md`。控制器配置修改后必须运行：

```bash
python3 scripts/validate_controller_mapping.py
```

当前 Humble 基线完成 5 个包构建，测试汇总为 16 项、0 错误、0 失败、1 项版权检查跳过。该结果只证明构建和静态测试通过，不代表 Gazebo 动态控制、真机执行或视觉精度已经验收。

Robot profile 修改后必须运行：

```bash
scripts/zzx_profile validate
scripts/zzx_profile validate zzx_sim_mobile_6dof --automatic
python3 -m unittest discover -s tests -p 'test_robot_profiles.py' -v
```

比赛 profile 在手眼/TCP 标定完成前会主动拒绝 `--automatic`，避免占位外参进入自动控制。

## 开发约定

- Python 用于视觉、训练、标定和离线分析；C++ 用于执行监控、规划、底盘对准和任务运行时。
- 通用数据优先使用 ROS 2 标准消息；长时间运动使用 Action。
- 任务成功必须来自执行反馈和物理结果验证，不能用固定 `sleep` 推断。
- 仿真与真机分别报告，未实测能力必须标为规划项。
- 提交描述使用中文，并写清改动后的可观察行为。

上游 LeoRobot 代码采用 MIT License，二次开发必须保留原版权和许可。第三方模型、SDK 和依赖按各自许可证单独核验。
