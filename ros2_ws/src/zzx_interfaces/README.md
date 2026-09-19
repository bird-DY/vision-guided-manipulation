# zzx_interfaces

Typed ROS 2 contracts shared by Python and C++ nodes in the Zzxrobot system.
This package contains interface definitions only. It must not contain runtime
business logic or hardware-specific code.

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

- `LocateObject`: find an object and estimate a 3D pose.
- `AlignBase`: visually align the mobile base with a detected target.
- `MoveArm`: plan or execute a joint-space or Cartesian arm target.
- `ControlHand`: command a gripper or dexterous hand.
- `ExecuteTask`: run a high-level task with progress and recovery feedback.
- `GetSystemStatus`: aggregate subsystem readiness and diagnostics.
- `SetControlMode`: select a hardware or controller operating mode.

## Recommended action names

```text
/zzx/perception/locate_object
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

## Migration from the upstream prototype

```text
Float32MultiArray /target_position -> LocateObject result
String /wheel_control              -> AlignBase goal and feedback
String /navigate_over              -> NavigateToPose result
String /action_over                -> ControlHand or ExecuteTask result
String /vision_on                  -> LocateObject goal
String /voice_commands             -> TaskCommand
```

## Design rules

1. Long-running or cancellable operations are actions, not services.
2. Services are limited to short configuration and status requests.
3. Physical quantities include units in field names when ambiguity is likely.
4. Poses always carry a frame through a header or stamped message.
5. Every action reports success, an error code, and a human-readable message.
6. Implementations may move between `rclpy` and `rclcpp` without changing the
   public interface.
