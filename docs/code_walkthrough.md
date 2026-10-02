# B05：从配置、接口到 C++ 假机械臂

本次实现位于 `ros2_ws/src/zzx_execution`。B01-B04 建立了环境、配置和公共消息；B05 增加可运行的运动模拟。它没有 HTTP 客户端，不读取现场 IP，也不连接 Gazebo。

## 项目的层次

| 目录 | 职责 | 读代码时问什么 |
| --- | --- | --- |
| configs/robots | 机器人关节、设备和能力声明 | 这台机器人支持什么？ |
| zzx_tools/profile.py | 配置检查 | 配置错误能否在启动前发现？ |
| ros2_ws/src/zzx_interfaces | 消息、服务、Action 定义 | 各模块交流什么数据？ |
| ros2_ws/src/zzx_contracts | Python 消息语义校验 | 数据能序列化，是否也符合业务规则？ |
| ros2_ws/src/zzx_execution | C++ 后端与后续执行监控 | 收到目标后怎么运动、怎么判断失败？ |
| ros2_ws/src/Zzxrobot | 上游模型、MoveIt、导航和原型节点 | 哪些现有能力可以复用？ |

## 本次代码阅读顺序

1. `include/zzx_execution/fake_arm_backend.hpp`：公共 API、故障和状态枚举。头文件告诉使用者可以做什么。
2. `test/test_fake_arm_backend.cpp`：先看 `MapsNamesAndCompletes` 和 `StuckNeverCompletesAndCanCancel`，理解正常与异常预期。
3. `src/fake_arm_backend.cpp`：实现按名重排、线性插值、故障注入、取消。
4. `src/fake_arm_node.cpp`：把 ROS 话题、服务和计时器转换为后端调用。
5. `test/test_ros_smoke.py`：启动真实 C++ 进程，经过 DDS 验证反馈和取消，结束时回收进程。

### 为什么核心不依赖 ROS

`advance(0.5)` 表示把模拟时间推进半秒，无须等待半秒。相同目标、故障参数和时间步长的结果完全一致，因此不需要随机种子。ROS 节点用 steady_clock 的真实间隔调用 advance；实际话题到达时间仍受调度影响。

运动方程是 `q = q_start + (q_target - q_start) * ratio`。它只用于测试执行监控，不模拟惯性、力矩、碰撞或真实限位。

`execute` 要求完整关节集合，但允许名称顺序变化。错误目标抛出异常；忙碌和注入拒绝返回 false，且不覆盖正在执行的目标。`cancel` 固定当前模拟位置。反馈丢失时不返回隐藏位置，原更新时间保留；节点停止发布反馈。

注意：后端内部 `succeeded` 不能直接当成未来 Action 的成功。B06/B07 必须依据新鲜反馈、误差和稳定时间判定；断反馈后即使内部运动结束，也不能报告执行成功。

## 在 VS Code 中操作

在 WSL 执行 `cd ~/vision_guided_manipulation && code .`，打开项目根目录。`.vscode` 提供建议扩展、构建任务、测试任务和 GDB 调试配置；扩展需要在 WSL 窗口中安装，本次未自动安装扩展。

- Ctrl+Shift+B：构建 B05。
- 命令面板 `Tasks: Run Task` → `B05: test`：运行 C++ 单元测试。
- `B05: ROS smoke test`：运行真实 ROS 通信测试。
- 在 `FakeArmBackend::execute`、`advance`、`cancel` 设置断点，选择 `B05: debug backend tests` 后 F5。需要 WSL 中有 `/usr/bin/gdb`。
- F10 单步越过，F11 进入函数；查看 `positions_`、`elapsed_`、`state_` 如何改变。

## 手动运行

每个新终端先执行：

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=86
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

终端 A 启动：

```bash
ros2 run zzx_execution fake_arm_node --ros-args --params-file src/zzx_execution/config/fake_arm.yaml
```

终端 B 观察：

```bash
ros2 topic echo /fake_arm_backend/joint_states
```

终端 C 发送测试目标：

```bash
ros2 topic pub --once /fake_arm_backend/target sensor_msgs/msg/JointState "{name: [joint1, joint2, joint3, joint4, joint5, joint6], position: [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]}"
ros2 service call /fake_arm_backend/cancel std_srvs/srv/Trigger '{}'
```

默认运动只需一秒，手动取消可用 slow 模式。停止节点后重启并覆盖参数：

```bash
ros2 run zzx_execution fake_arm_node --ros-args -p fault:=slow
```

| fault | 行为 |
| --- | --- |
| normal | duration_seconds 内完成线性插值 |
| slow | 耗时变为五倍 |
| reject | 拒绝目标，位置不变 |
| stuck | 推进到 fault_after_seconds 后停住，保留执行中状态 |
| feedback_loss | fault_after_seconds 后停止发布反馈，内部运动仍继续 |

参数在启动时读取，修改 YAML 后重启节点。stuck 要在运动完成前触发，fault_after_seconds 应小于 duration_seconds。

## 验证与后续

核心测试覆盖启动静止、名称映射、拒绝、慢速、卡住、取消、断反馈、忙碌、非法输入和确定性回放。ROS 通信测试覆盖正常目标、慢速中取消和断反馈。两者分别验证算法行为与消息链路。

这台 WSL 的默认 Fast DDS 在 localhost-only 测试中未发现对端，切换现有 Cyclone DDS 后三项通信测试通过。因此测试与上述终端流程固定使用 Cyclone DDS；这不代表其他机器的 Fast DDS 必然有问题。不要让测试客户端和服务端使用不同的域号或不同的本机通信设置。

B05 的测试话题不是正式 MoveArm Action。B06 会新增 Action server，负责目标校验和资源占用；B07 再实现反馈时效、误差收敛、稳定时间、取消确认。真实关节限位、碰撞规划和动力学尚未在这个假后端中实现。

建议每次改动先读测试预期，再看实现差异，最后执行验收。提交使用中文，保留小步、可回退的版本，方便对照每一步设计。
