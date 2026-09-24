# zzx_interfaces 0.2

Zzxrobot 的 Python/C++ 公共 ROS 2 契约。该包只包含 IDL；字段语义校验位于
`zzx_contracts`，硬件与任务业务逻辑不得进入接口包。

## Standard interfaces reused directly

- RGB and depth streams: `sensor_msgs/msg/Image`
- Camera calibration: `sensor_msgs/msg/CameraInfo`
- Joint feedback: `sensor_msgs/msg/JointState`
- Base velocity: `geometry_msgs/msg/Twist`
- Navigation: `nav2_msgs/action/NavigateToPose`
- Joint trajectory execution: `control_msgs/action/FollowJointTrajectory`
- TF transforms: `tf2_msgs` and `geometry_msgs`
- Action cancellation: standard ROS 2 action cancellation

## Project interfaces

- `ObjectTarget`: 带图像来源、标定 ID、分离有效性和不确定度的三维目标。
- `BackendCapabilities`: 后端经过验证的能力，不可把“可能支持”填成 `true`。
- `ErrorStatus`: 所有 Action 和 Service 共用的稳定错误码。
- `ExecutionStatus`: 操作 ID、进度、阶段和统一终态。
- `LocateObject`: 发现一次目标；连续跟踪使用独立目标话题。
- `AlignBase`: 以目标为参照闭环对准底盘。
- `MoveArm`: 严格互斥的关节或笛卡尔目标。
- `ControlHand`: 按名称稀疏更新夹爪或灵巧手关节。
- `ExecuteTask`: 执行可取消、可恢复的高层任务。

## Recommended action names

```text
/zzx/perception/locate_object
/zzx/perception/object_target
/zzx/base/align_to_target
/zzx/manipulation/move_arm
/zzx/manipulation/control_hand
/zzx/task/execute
```

Recommended status endpoints:

```text
/zzx/status
/zzx/get_system_status
/zzx/set_control_mode
```

## Field semantics

### Time, frames and units

- `ObjectTarget.image_header` 是实际参与估计的彩色图像时间和光学坐标系，时间不可为零。
- `ObjectTarget.header` 是三维估计时刻和输出坐标系；位置或姿态有效时 `frame_id` 必填。
- 坐标遵循 ROS REP-103，长度为 m、角度为 rad、速度为 m/s 或 rad/s。
- `roi` 是原始 `image_width x image_height` 图像中的像素区域，不允许越界。
- `calibration_id` 必须能追溯到与分辨率、D2C 和外参匹配的 manifest。
- ROS 基础类型虽然默认为 0，但 0 不代表业务默认值；必填字段由 `zzx_contracts` 拒绝空值。

### Quality and validity

- `confidence` 与 `TargetQuality.score` 范围均为 `[0, 1]`。前者是识别置信度，后者是可用于动作的综合质量。
- `position_valid` 与 `orientation_valid` 独立。对称物体可以只有位置，不得伪造完整姿态。
- `position_std_m`、`orientation_std_rad` 是标准差；对应分量无效时不参与动作判断。
- `depth_valid=true` 时 `depth_m` 必须有限且大于 0。

### Commands

- `MoveArm.target_type` 只能选一种目标。关节目标按名称映射且数组等长；姿态目标必须有 frame 且四元数归一化。
- 三种 scaling 范围均为 `(0, 1]`；0 不表示使用默认速度。所有 Action 的 `timeout` 必须大于 0。
- `ControlHand` 允许只给一个或几个关节；未列出的关节必须保持当前值。
- 后端不支持 `max_effort` 或 `stop_on_contact` 时返回 `UNSUPPORTED`，不能静默忽略。
- `TaskCommand.schema_version` 当前必须为 2；扩展键必须唯一，消费节点还需按任务白名单验证。

## Action terminal states

| Action terminal state | `execution.state` | `error.code` |
| --- | --- | --- |
| succeeded | `STATE_SUCCEEDED` | `OK` |
| aborted | `STATE_FAILED` | non-`OK` |
| canceled | `STATE_CANCELED` | `CANCELED` |
| backend outcome unknown | `STATE_NEEDS_RECONCILIATION` | `RESULT_UNKNOWN` |

`success=true` 仅能与第一行同时出现。`plan_only=true` 的成功结果必须将
`execution.stage` 写为 `planned_only`，不得表述为已执行。目标到达但未满足 settling 条件时仍为执行中。

## Migration from 0.1 to 0.2

| 0.1 | 0.2 | Migration |
| --- | --- | --- |
| `ObjectTarget.pose_valid` | `position_valid` + `orientation_valid` | 分别判断位置和姿态 |
| 目标只有 `header` | `header` + `image_header` | 保存采集帧时间和输出帧 |
| 无标定来源 | `calibration_id` | 填写 manifest ID |
| 无质量结构 | `TargetQuality` | 输出方法、样本数和不确定度 |
| `error_code` + `message` | `ErrorStatus error` | 使用共享常量和 retryable |
| Action 自有 `stage/progress/detail` | `ExecutionStatus execution` | 统一操作 ID、状态、进度和阶段 |
| `TaskCommand` 无版本/profile | `schema_version=2` + `robot_profile_id` | 明确解析版本和机器人能力 |
| `GetSystemStatus.message` | `GetSystemStatus.error` | 使用共享错误对象 |

Upstream prototype migration:

```text
Float32MultiArray /target_position -> LocateObject result
String /wheel_control              -> AlignBase goal and feedback
String /navigate_over              -> NavigateToPose result
String /action_over                -> ControlHand or ExecuteTask result
String /vision_on                  -> LocateObject goal
String /voice_commands             -> TaskCommand
```

## Validation and build

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to zzx_contracts
source install/setup.bash
colcon test --packages-select zzx_interfaces zzx_contracts
colcon test-result --verbose
ros2 run zzx_contracts target_contract_monitor
```

`zzx_contracts.validation` 是 Python 参考实现。C++ 执行节点必须实现等价约束，并通过相同反例数据。

## Design rules

1. Long-running or cancellable operations are actions, not services.
2. Services are limited to short configuration and status requests.
3. Physical quantities include units in field names when ambiguity is likely.
4. Poses always carry a frame through a header or stamped message.
5. Every action reports `success`, shared `ErrorStatus`, and `ExecutionStatus`.
6. Implementations may move between `rclpy` and `rclcpp` without changing the
   public interface.
