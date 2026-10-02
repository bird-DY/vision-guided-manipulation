# B06：统一 MoveArm Action 的第一版实现

本文记录 B06 的初始实现；停止与反馈处理已在 [B07 执行监控](execution_monitor.md) 中升级，以 B07 为准。

本阶段只运行本地假后端，不访问比赛 HTTP 接口、不控制真实机械臂。它验证执行契约，不代表真机性能或碰撞安全已验收。

## 代码阅读顺序

1. `ros2_ws/src/zzx_interfaces/action/MoveArm.action`：已有公共接口，分目标、结果和反馈；本次没有改变字段。
2. `ros2_ws/src/zzx_execution/src/fake_arm_backend.cpp`：已有确定性关节插值和故障注入，不依赖 ROS。
3. `ros2_ws/src/zzx_execution/src/move_arm_server.cpp`：新增 C++ Action 服务。
4. `ros2_ws/src/zzx_execution/test/test_move_arm_action.py`：Python 客户端通过真实 DDS 测试 C++，并非用 Python 替代执行层。

阅读服务按 `构造函数 → accept → validate → tick → finish`：

- 构造函数注册服务、反馈发布器和定时器，不下发运动。
- Goal 回调占用 `reserved_`，拒绝第二个目标；这是单进程互斥，不是分布式资源锁。
- `accept` 生成 UUID 操作 ID，`validate` 按关节名字映射位置，不能依赖输入排列。
- `tick` 每约 20 ms 推进假后端，用单调时钟计算超时，不用阻塞 sleep 等待完成。
- 后端完成且所有关节误差满足容差后进入 SETTLING，持续稳定才成功。
- `finish` 同步 ROS Action 终态、success、error 和 execution.state，释放占用。

当前使用单线程 executor 和默认互斥回调组，无 detached thread。将来引入并行回调时需要重新检查共享状态同步。

## 契约与限制

- 关节目标必须完整列出已配置的全部关节；未知名、重复名、非有限值、越界值和数组长度错误返回 INVALID_ARGUMENT。
- 不支持 JointState 中的 velocity/effort；不允许同时填写关节与姿态目标。
- 两种 scaling 都必须有限且在 `(0,1]`，timeout 必须大于零。
- 非法目标在传输层接收后立即 abort，以便返回结构化错误，但绝不调用后端。
- 忙碌或失联锁定时直接拒绝新 Goal，无 Result；客户端必须检查 accepted。
- pose 目标返回 UNSUPPORTED，等待 MoveIt 后端，不允许伪成功。
- plan_only 只做假后端目标范围检查，不运动，stage 严格为 `planned_only`。不包含碰撞检测、IK 或真实轨迹规划。
- reject 故障返回 BACKEND_REJECTED；stuck 超时返回 TIMEOUT。
- B07 起反馈持续丢失会导致停止无法确认，返回 STOP_UNCONFIRMED；最终关节数据留空并锁定新动作。
- B07 起取消需新鲜反馈确认停稳后才返回 CANCELED。这不是硬件急停或真机停止证明。
- progress 是阶段值（0、0.5、0.95、1），不是路径完成比例；最终 pose 没有计算，不得用于定位。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| joint_names | joint1 至 joint6 | 假模型，不是比赛七轴命名 |
| initial_positions | 全 0 | rad |
| lower_limits / upper_limits | 每轴 -π / +π | 仅用于测试，不可套用真机 |
| duration_seconds | 1.0 | scaling=1 的插值时长 |
| fault | normal | normal、slow、reject、stuck、feedback_loss |
| fault_after_seconds | 0.25 | 模拟时间的故障发生点 |
| position_tolerance | 0.001 | rad |
| settling_seconds | 0.1 | 连续稳定时间，秒 |

模拟时间按 `min(velocity_scaling, sqrt(acceleration_scaling))` 缩放。这只是延长模拟运动时间，不是真实速度/加速度约束；超时使用真实单调时间。参数仅在启动时读取，调整需要重启。

## 在 VS Code 运行

在 Ubuntu 中执行：

```bash
cd ~/vision_guided_manipulation
code .
```

确认左下角是 WSL。在“终端 → 运行任务”选择：

- `B06: Action test`：先构建，再自动测试并清理测试服务。
- `B06: run fake Action server`：先构建，再持续运行服务，Ctrl+C 结束。

另开 WSL 终端发送目标：

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 action send_goal /zzx/manipulation/move_arm zzx_interfaces/action/MoveArm \
  '{target_type: 1, joint_target: {name: [joint1, joint2, joint3, joint4, joint5, joint6], position: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]}, velocity_scaling: 1.0, acceleration_scaling: 1.0, plan_only: false, timeout: {sec: 5, nanosec: 0}}' --feedback
```

应该看到 executing、settling 和成功结果。反馈话题为 `/move_arm_server/joint_states`。
将 plan_only 改为 true：stage 应为 planned_only，位置不变。
将 joint1 改成未知名字：应得到 INVALID_ARGUMENT，不运动。
启动时增加 `--ros-args -p fault:=stuck` 可测试卡住超时；不要在同一命名空间同时启动多个同名服务。

## 验收与下一步

新增八个 Action 集成用例：名称乱序映射与稳定确认、仅规划、十二类非法输入、pose 不支持、并发拒绝和取消后复用、卡住超时、反馈丢失锁定、后端拒绝。
测试类共享 ROS context，显式销毁 ActionClient，使用本地 Cyclone DDS。

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to zzx_execution
source install/setup.bash
colcon test --packages-select zzx_execution
colcon test-result --test-result-base build/zzx_execution --verbose
```

2026-10-02 单包回归汇总 24 项、0 错误、0 失败、0 跳过；含 21 个实际用例和 3 个测试容器记录。未验收 VS Code GUI、Gazebo 动态控制或真机。

B07 已提取独立执行监控，详见对应文档。之后再逐步接 MoveIt/真实控制器。断电恢复、跨进程资源所有权和碰撞场景不在 B06 范围。
