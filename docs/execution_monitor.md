# B07：执行监控与停止确认

## 为什么需要这一层

比赛中已经遇到“规划成功但执行失败”。同样，控制器返回成功不代表实际位置到达，取消请求被接受也不代表机械臂停下。因此任务调度不能只判断 HTTP 200 或 Action 的请求接收状态。

本阶段在假后端实现：

```text
MoveArm Action 服务：接收目标、独占后端、对外反馈、组织停止流程
    ↓                       ↑
FakeArmBackend：执行/取消    ExecutionMonitor：依据反馈判断执行结果
```

这不是完整工业安全控制器，不替代硬件急停；没有连接真机，也没有增加真实动力学模型。

## 从哪些文件开始看

| 文件 | 阅读重点 |
| --- | --- |
| `ros2_ws/src/zzx_execution/include/zzx_execution/execution_monitor.hpp` | 配置、控制器结果、监控判定；与 ROS 无关 |
| `ros2_ws/src/zzx_execution/src/execution_monitor.cpp` | start、request_stop、update；调用者注入单调时间，便于确定性测试 |
| `ros2_ws/src/zzx_execution/src/move_arm_server.cpp` | 假后端反馈适配、begin_stop、判定转 Action 终态 |
| `ros2_ws/src/zzx_execution/test/test_execution_monitor.cpp` | 14 个纯 C++ 用例，直接看输入时间与预期状态 |
| `ros2_ws/src/zzx_execution/test/test_move_arm_action.py` | 10 个跨进程 Action 用例，验证真实 DDS 与终态 |

监控器复用原有库构建，不需要安装新插件或依赖。Python 仍负责测试客户端；动作与监控逻辑在 C++。

## 成功的四个条件

1. 控制器结果必须为 succeeded，不能只是仍在运行。
2. 每个关节位置误差必须在 position_tolerance 内。
3. 根据相邻新鲜反馈计算的每轴速度必须在 velocity_tolerance 内。
4. 上述条件连续满足 settling_seconds。越界、运动、样本间隔过长都会重新计时。

重复时间戳不会累计新证据；未来时间、倒退时间戳、NaN、尺寸错误、超龄或不可用反馈不能用于成功判定。反馈时钟必须与监控器的单调时钟同域。

连续旋转关节使用 `remainder(target - actual, 2π)`，例如跨 ±π 的微小变化不会被认为转了整圈。普通有限角关节使用直接差值。
注意：这只改变误差/速度判定，不改变假后端插值路径，也不是 continuous 关节的最短路径规划。

## 停止流程

```text
取消 / 执行超时 / 控制器失败 / 反馈丢失
  → begin_stop：只调用一次底层 cancel，保留最初原因
  → 继续占用资源，不接收新动作
  → 新鲜连续反馈 + 低速度 + 控制器已取消或已成功
  → 停止确认后返回最初错误码
```

- 正常取消确认后：ROS canceled，错误码 CANCELED。
- 超时确认停止后：ROS aborted，错误码 TIMEOUT。
- 控制器失败确认停止后：ROS aborted，错误码 EXECUTION_FAILED。
- 停止期限内无法确认：ROS aborted，错误码 STOP_UNCONFIRMED，并拒绝后续目标。
- 反馈持续丢失也无法确认停止，因此最终返回 STOP_UNCONFIRMED；message 保留反馈丢失原因。
- 反馈失联即锁定服务，即使之后恢复并确认停止也不自动解锁。当前仅假服务重启恢复，真实硬件应另做状态核对流程。

取消在到位稳定阶段发生时，也必须走停止确认，不得再进入 succeeded。
重复取消不延长停止期限。执行超时是进入停止流程的时刻，不是立即强制返回；最终结果最多额外等待 stop_timeout_seconds 加调度开销。

## 时间与参数

新增启动参数：

| 参数 | 默认值 | 单位/含义 |
| --- | --- | --- |
| continuous_joints | 与关节数相同的全 false 数组 | 按 joint_names 顺序 |
| velocity_tolerance | 0.01 | rad/s，有限差分停止/稳定阈值 |
| feedback_timeout_seconds | 0.2 | 秒，反馈允许的最大年龄及连续采样间隔 |
| stop_timeout_seconds | 0.5 | 秒，必须大于 settling_seconds |

沿用 position_tolerance=0.001 rad、settling_seconds=0.1 s。所有阈值必须为有限正数，仅启动时读取。

假后端的源时间戳随模拟时间推进；服务仅在源时间戳前进时记录新的单调接收时间，避免每次读取旧帧都给它刷新年龄。JointState 发布也只发生在新样本到达时。
真实后端接入时必须验证采样序号/源时间映射和传输延迟；不能只把 HTTP 轮询返回时间当成硬件采样时间。

目前使用关节位置有限差分估计速度，不能检测两个采样之间未被观察到的运动。阈值、采样频率与稳定窗口必须在具体硬件上另行验证。

## 运行与验收

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to zzx_execution
source install/setup.bash
colcon test --packages-select zzx_execution
colcon test-result --test-result-base build/zzx_execution --verbose
```

单独运行监控器测试，可逐个修改输入理解判定：

```bash
./build/zzx_execution/test_execution_monitor
```

VS Code 现有 `B05: test` 任务会运行包含 B07 的整个执行包回归；`B06: Action test` 会运行扩展后的 10 个 Action 用例，无需增加插件。

新增 fake fault：

- controller_failure：控制器在指定时间报失败，服务必须停止并返回 EXECUTION_FAILED。
- cancel_reject：底层拒绝取消。Action 测试减慢动作，保证停止窗口内仍未结束，验收 STOP_UNCONFIRMED 与新目标拒绝。

既有 feedback_loss 用例已更新：不能再用“已发 cancel”当成“确认停下”。

2026-10-02 验收：Debug 构建通过，执行包回归汇总 41 项、0 错误、0 失败、0 跳过。其中实际用例为 24 个 C++ 和 13 个 ROS 通信用例，其余 4 项是测试容器记录。未进行真机或 VS Code GUI 验收。

## 后续边界

B08 将做操作 ID、幂等与结果持久化，解决客户端重试、结果丢失和重启恢复。本阶段仍是单进程互斥；没有真机恢复接口、硬件级停止证明、碰撞检测或运行时动态调参。
