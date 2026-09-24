# Robot profile 使用说明

Robot profile 把仿真六轴移动机械臂与比赛七轴左臂/O10 灵巧手彻底隔离。任务代码只消费已经验证的 profile，不根据数组长度或 URL 猜测机器人类型。

## 命令

```bash
cd ~/vision_guided_manipulation
scripts/zzx_profile list
scripts/zzx_profile show zzx_sim_mobile_6dof
scripts/zzx_profile validate
scripts/zzx_profile validate zzx_sim_mobile_6dof --automatic
```

普通 `validate` 检查 Schema、DOF、关节唯一性、工具注册和后端地址。`--automatic` 进一步检查自动运动许可及标定状态。

当前比赛 profile 的手眼/TCP 仍是占位状态，因此下面的命令必须失败：

```bash
scripts/zzx_profile validate competition_left_7dof_o10 --automatic
```

这不是程序故障，而是防止零值外参或未验证 TCP 进入自动控制的安全门。完成真实标定后，需要添加可追溯 manifest、把 `calibration.status` 改为 `validated`、设置 `transform_available: true`，并经过独立验证后才能打开 `automatic_motion_allowed`。

## 能力声明原则

- `trajectory: true` 表示后端接受带时间参数的完整轨迹，不等同于能发送单个关节目标。
- `pose_target: true` 表示后端有经过契约测试的笛卡尔目标能力。
- `force_control: true` 只有在硬件、协议和闭环反馈均证实时才能填写。
- 现场 HTTP 机械臂目前只声明关节目标、规划检查、取消和示教模式，不声明轨迹透传、笛卡尔目标或力控。
- O10 当前只声明位置命令，不把电流反馈误写成闭环力控能力。

## 新增 profile

1. 在 `configs/robots` 新增 YAML，不复制并修改运行代码。
2. 在 `configs/tools` 注册工具、关节列表和 TCP frame。
3. 运行普通和自动模式校验。
4. 为非法关节、错误向量、未知工具和标定状态补测试。
5. 真机 profile 必须记录设备型号、固件、接口能力证据和标定 manifest。

Schema 使用 `additionalProperties: false`。新增字段应先修改 Schema、文档和测试，不能依靠 YAML 中无人消费的自由字段。
