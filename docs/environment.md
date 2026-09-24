# 开发环境基线

本文记录 `vision_guided_manipulation` 当前可复现基线。它描述已经验证的 Humble 环境，不代表最终部署平台；Jazzy/Harmonic 迁移按实施清单 B36 单独进行。

## 1. 已验证平台

| 项目 | 版本 |
|---|---|
| Windows 宿主 | WSL 2 |
| Ubuntu | 22.04.5 LTS |
| Linux kernel | 6.6.87.2-microsoft-standard-WSL2 |
| ROS 2 | Humble，`ros-humble-ros-base 0.10.0` |
| MoveIt 2 | 2.5.9 |
| Navigation2 | 1.1.20 |
| ros2_control | 2.54.0 |
| gazebo_ros_pkgs | 3.9.0 |
| Cartographer ROS | 2.0.9002 |
| Python | 3.10 |
| C++ | C++17，GCC 11 基线 |

APT 小版本会随 ROS 2 Humble 仓库安全更新变化。每次发布应把 `dpkg-query` 输出、Git commit 和构建日志保存到发布工件，而不是仅依赖此表。

## 2. 安装

```bash
cd ~/vision_guided_manipulation
chmod +x scripts/bootstrap_ubuntu.sh scripts/doctor.sh
./scripts/bootstrap_ubuntu.sh
./scripts/doctor.sh
```

默认安装 ROS、MoveIt、Nav2、Gazebo、Cartographer 和构建工具，不安装 CUDA 视觉环境。需要复现当前 CUDA 12.8 视觉环境时使用：

```bash
INSTALL_VISION_STACK=1 ./scripts/bootstrap_ubuntu.sh
```

`requirements/vision-cu128.lock.txt` 是当前 WSL 虚拟环境的版本快照，只适用于兼容 CUDA 12.8 的 NVIDIA 环境。CPU、Jetson 或其他 CUDA 版本必须维护独立锁文件，不能直接复用。

## 3. 构建和测试

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

控制器配置在构建前可独立验证：

```bash
cd ~/vision_guided_manipulation
python3 scripts/validate_controller_mapping.py
```

## 4. 包职责

- `mybot_description`：模型、网格、Gazebo world 和 Gazebo 启动入口。
- `mybot`：唯一权威 MoveIt 配置，以及机械臂/夹爪控制器映射。
- `zzx_interfaces`：跨 Python/C++ 的 ROS 2 接口，不包含运行逻辑。
- `mybot_cartographer`：上游 Cartographer 配置与启动入口。
- `mybot_navigation2`：上游 Nav2 配置与启动入口。

纯模型 `six_arm.urdf` 不再嵌入控制器。Gazebo 使用 `six_arm.gazebo.xacro` 追加 `GazeboSystem`，MoveIt 使用 `mybot/config/six_arm.urdf.xacro` 追加 mock ros2_control，避免同一 `robot_description` 出现两套 ros2_control。

## 5. 当前边界

- WSL 适合开发、构建和仿真，不作为 Gemini335 USB 时延或真机控制实时性的最终验收环境。
- Gazebo Classic 已进入维护终止状态；当前只用于复现上游基线。
- `Zzxrobot` 中 Python 节点仍是上游原型，尚未具备新的取消、反馈监控和幂等执行保证。
- 比赛机械臂和 O10 灵巧手的 HTTP 接口尚未由本仓库的 ROS 2 后端统一接管。
- CUDA 锁文件包含 Ultralytics AGPL 依赖；商业发布前必须单独完成模型和依赖许可审计。

## 6. 发布时必须保存

1. `git rev-parse HEAD` 与 `git status --short`。
2. `scripts/doctor.sh` 输出。
3. `dpkg-query -W 'ros-humble-*'` 的相关软件包版本。
4. Python 锁文件、模型 SHA-256、配置 SHA-256 和标定 ID。
5. `colcon build/test/test-result` 日志。
6. 仿真随机种子；真机则记录机器人、工具、场景和操作者。
