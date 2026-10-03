# B11：统一启动、生命周期与命令所有权

## 本阶段的行为变化

HTTP 适配节点不再启动即接收动作，改为标准 ROS 2 LifecycleNode：

```text
unconfigured → configure → inactive（只读设备检查）
  → 操作人确认 ready → activate → active（允许目标）
  → deactivate / 设备掉线 → inactive（拒绝新目标，请求停止进行中的动作）
```

恢复通信只恢复 ready，不自动 activate。进程重启仍为 unconfigured，不重放运动。
清理操作不能清除未知结果锁定；仍有动作时 cleanup 被拒绝。inactive 表示入口关闭，不证明所有物理运动已停止，应继续观察 busy、latched 与 Action 最终结果。

## 代码入口

| 文件 | 职责 |
| --- | --- |
| `zzx_bringup/launch/stack.launch.py` | 选择 fake/http/sim；不包含旧运动脚本；受管后端退出后关闭该启动组 |
| `zzx_bringup/config/profiles.json` | 明确选择唯一执行链，标识 sim 暂无任务入口 |
| `zzx_http_backend/node.py` | 生命周期、后台就绪探测、掉线停用、动作准入门控 |
| `zzx_http_backend/ownership.py` | 基于 flock 的本机协作所有权 |
| `zzx_execution/src/move_arm_server.cpp` | C++ 假服务使用同一所有权文件协议 |
| `Zzxrobot/mybot_description/mybot_description/move_arm.py` | 旧脚本默认禁用，显式启用也必须取锁；移除启动时夹爪张开和回零命令 |

各路径均在 `ros2_ws/src` 下。修改旧脚本只涉及启动安全和所有权，不重写其旧抓放流程。

## 三个 profile 的实际能力

| profile | 启动内容 | 任务入口 |
| --- | --- | --- |
| fake | C++ MoveArm 假服务 | 进程初始化成功即能接收假动作；不具有硬件副作用 |
| http | 两个环回 HTTP 假服务 + 生命周期适配器 | 必须 configure、ready、显式 activate |
| sim | 既有 Gazebo、模型和控制器基础环境 | 不启动自主任务 Action，也不启动旧 move_arm.py |

sim 不是“Gazebo 已接通新执行器”的声明。MoveIt 执行适配尚未完成，不用假服务冒充仿真反馈。sim 沿用上游固定控制器命名与 namespace，强制 ownership_id=zzxrobot；namespace 参数只作用于 fake/http 的 Action 节点。

HTTP 仍拒绝非环回地址，现场 profile 的 automatic_motion_allowed=false 未改变。

## 就绪检查

configure 后，在工作线程里循环读取机械臂 status/motors 与灵巧手 status：

- 机械臂能返回完整、有限的七轴位置，moveit_available=true。
- 电机 has_feedback=1、fault=0。
- 灵巧手 connected=true 且有合法十维归一化位置。

ready 表示通信与基础反馈可用，不是碰撞安全、标定有效或电机已使能的证明。使能仍在显式动作的 prepare 阶段进行。
激活时还要求机械臂未运动、没有占用或未知结果锁定。探测过期、失败、检测到旧 arm_controller，或者后端结果锁定，会关闭入口并停用。
工作线程检查约每 0.25 秒启动一次，探测预算 1 秒，旧结果超过 1.5 秒不能用于接收新 Goal。configure/cleanup 有代次校验，旧探测结果不会使新配置直接就绪。

`~/get_status` 返回 active、ready、reason、busy、latched。该查询仍不阻塞网络。设备掉线后无法确认停止时，不能因为 inactive 就认为设备已停稳。

## 单命令所有权的范围

默认文件：`~/.local/state/zzxrobot/owners/zzxrobot.lock`。

- C++ 假服务、HTTP 适配器、sim 启动器和显式启用的旧脚本，默认竞争同一把锁。
- HTTP 还按规范化的端点地址取得独立锁，即使换 ownership_id，也不能在本机重复控制同一 HTTP 端点。
- 锁持续到进程退出；inactive/cleanup 不释放，避免第二进程趁停用接管。
- fake/http 可用不同 ownership_id 隔离真正不同的测试资源，不能借此绕过同一机器人的所有权。
- ROS graph 检查是补充诊断；真正互斥靠文件锁，不依赖发现时序。

这是同一用户、同一 Linux 主机上的协作锁，不是硬件安全机制或分布式锁。其他主机、不同用户、直接 curl、直接发布 JointTrajectory、未遵守协议的外部客户端仍能绕过。不要删除锁文件来“解锁”，已有持有者仍可能在执行。
进程被杀后 OS 会释放文件锁，但物理动作结果仍可能未知；B08 数据库核对流程必须保留。

旧脚本需要 ZZX_ENABLE_LEGACY=1 才能显式使用，资源 ID 由 ZZX_ROBOT_ID 指定；本阶段不建议在新执行链旁启用。即使启用，构造函数也不再张开夹爪或回零。

## 在 VS Code / WSL 中运行

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to zzx_bringup
source install/setup.bash
export ROS_DOMAIN_ID=90
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch zzx_bringup stack.launch.py profile:=http
```

默认命名空间 robot，HTTP 端口 18087/18088。端口占用时指定 arm_port/hand_port 的两个空闲端口，不结束已有程序。
另开同样 source 和环境变量的终端：

```bash
ros2 lifecycle get /robot/http_action_backend
ros2 lifecycle set /robot/http_action_backend configure
ros2 service call /robot/http_action_backend/get_status std_srvs/srv/Trigger '{}'
```

确认返回 ready=true 后，再执行：

```bash
ros2 lifecycle set /robot/http_action_backend activate
ros2 action send_goal /robot/zzx/manipulation/control_hand zzx_interfaces/action/ControlHand \
  '{position_unit: 1, joint_names: [index_pip], positions: [0.0], speed_scaling: 1.0, max_effort: 0.0, stop_on_contact: false, timeout: {sec: 5}}' --feedback
```

停用入口：

```bash
ros2 lifecycle set /robot/http_action_backend deactivate
ros2 service call /robot/http_action_backend/get_status std_srvs/srv/Trigger '{}'
```

确认进行中的动作已结束或进入明确核对状态后，再在 launch 终端 Ctrl+C。不要把 Ctrl+C 当作真实设备急停。
只运行 C++ 假后端可选 profile:=fake；查看仿真基础环境可选 profile:=sim。不要同时运行同一资源的多个 profile。

## 回归与未验收项

```bash
colcon test --packages-select zzx_bringup zzx_http_backend zzx_http_contracts zzx_task_runtime zzx_execution
colcon test-result --test-result-base build/zzx_bringup --verbose
colcon test-result --test-result-base build/zzx_http_backend --verbose
```

覆盖未激活拒绝、设备失联停用、通信恢复不自动激活、进程重启不重放、跨进程/跨语言所有权、新旧入口互斥，以及真实 launch 启停。
Gazebo GUI 和 sim 动态控制本阶段未实测；profile 选择及不启动旧任务入口有静态检查。真机、分布式所有权、恶意客户端和完整安全认证不在本阶段验收范围。

2026-10-03 验证记录：7 个依赖及应用包构建成功；bringup 5 项、HTTP 适配 16 项测试全部通过，无错误、失败或跳过。此前本轮相关回归中，HTTP 契约 17 项、持久化 10 项、执行包汇总 41 项（含测试容器记录）也全部通过。测试仅使用 WSL、本地 DDS 和回环 HTTP 假服务，没有操作真机。

下一步进入 B12：整理 RGB-D 数据清单和离线回放集，为视觉定位提供可重复验收的数据基础。
