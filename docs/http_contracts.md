# B09：比赛 HTTP 假服务与契约回归

## 目标与来源

把现场机械臂 8087、灵巧手 8088 的已知通信行为保存为可重复测试，避免后续 ROS 2 适配器重复踩坑。

本阶段核对了 Windows 比赛代码：

- `D:/Desktop/zhongkong/final/final_app/hardware/http_clients.py`
- `D:/Desktop/zhongkong/final/final_app/tools/arm_joint_debug.py`
- 本次比赛记录中的 Planning succeeded / Execution failed、prepare 后成功、right 错误标签等响应。

这些是项目现场观测，不是厂商完整协议规范。未实现的端点返回 404，不猜测或伪造支持能力。没有修改比赛现场代码，也没有请求 192.168.0.22。

## 新增模块

包目录：`ros2_ws/src/zzx_http_contracts`。

| 文件 | 作用 |
| --- | --- |
| `zzx_http_contracts/protocol.py` | 七轴名称顺序、十自由度手指名称、输入校验 |
| `zzx_http_contracts/fake.py` | 基于标准库 ThreadingHTTPServer 的环回假服务与故障场景 |
| `zzx_http_contracts/client.py` | 严格的参考客户端，用于验证协议，不是正式真机驱动 |
| `test/test_http_contracts.py` | 真实 socket 通信、状态变化、延迟断连及 B08 联动测试 |

无新增第三方 HTTP 依赖，不需要新插件。假服务只监听 127.0.0.1，参考客户端仅允许带端口的 `http://127.0.0.1`，禁用代理与重定向。正式 HTTP ROS 2 后端留待 B10。

## 固定下来的协议细节

### 机械臂

POST `/api/joints` 数组顺序必须为：

```text
肩 pitch、肩 roll、肩 yaw、肘 roll、肘 yaw、腕 pitch、腕 yaw
```

状态 JSON 的键曾按“肩 pitch、肩 roll、肘 roll、肘 yaw、肩 yaw、腕 pitch、腕 yaw”排列。必须按名称映射，不能直接 `list(values())`。

目标请求：mode=left_arm、left_joints 七元素、velocity_scaling、acceleration_scaling、plan_only、label。

准备流程：

```text
POST /api/teach_mode    {"enable": false}
POST /api/control_mode {"mode": "pos_vel"}
POST /api/enable       {}
GET  /api/motors       核对实际左臂的 enabled / has_feedback / fault
```

使能响应可能错误标为 right；只在这个已知端点容忍该标签，并检查实际左臂反馈，不能据此改成控制右臂。
假状态保留现场出现的 controllers.active=false、motor_error=1、fault=0。没有厂商位定义时，不把 motor_error=1 直接当成故障，也不把 active=false 单独当作执行不可用。

### 灵巧手

实现 GET `/api/status`、GET `/api/pose` 与 POST `/api/set_pos`。
模型为 O10 的十维位置接口；假序列号明确为 FAKE-O10，不伪装真实设备。

```text
thumb_roll, thumb_abad, thumb_mcp, index_abad, index_pip,
middle_pip, ring_abad, ring_pip, pinky_abad, pinky_pip
```

命名稀疏更新先读取十维状态，只改指定分量，再发送完整 position 数组。测试保证只改食指弯曲时其余九项不变。读改写不是后端原子操作，真机接入前必须通过单命令所有权消除并发覆盖。
数值限于 `[0,1]`，但不能从数值大小推断每个手指的伸屈方向；方向需要现场验证。力度、电流、set_pvc 和抓取接触检测不在本阶段范围。

## 场景与失败语义

| CLI scenario | 行为 |
| --- | --- |
| unprepared | 初始未准备：规划成功，执行失败；prepare 后成功 |
| prepared | 已准备的正常假执行 |
| execution_failure | 已准备，但执行仍 HTTP 400 / success=false |
| false_200 | HTTP 200 但 success=false，客户端必须报失败 |
| delayed | 假动作结束后延迟一秒才回复，可复现客户端超时 |
| disconnect | 假动作完成后断开连接，不返回结果 |

执行故障与延迟场景针对 arm；hand 用于位置读写契约，不模拟电机动力学或力反馈。
Python Scenario 还支持配置响应延迟、模拟运动时长和使能标签。所有时长限于 0～10 秒，测试用随机空闲端口，避免占用真实 8087/8088。

模拟运动中 moving=true；测试可查询状态或取消。正常结束才更新目标位置；模拟取消阻止目标更新。该模型没有插值、惯性或硬件停止证明。

响应分三类：

- 明确的成功确认：字段类型严格检查，但仍不是物理到位证明。
- ReportedFailure：后端明确报告失败，包括 HTTP 200 / success=false；不保证运动完全没有发生。
- UnknownOutcome：超时、断连、非 JSON、异常状态码或无法确认的响应；绝不能自动重发。

B08 联动测试展示：客户端收到 UnknownOutcome 时，假服务关节位置实际上已改变；同 task_id 重试不再发送，新 ID 也被未决状态阻挡。

## 启动与阅读顺序

在 WSL / VS Code 终端：

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to zzx_http_contracts
source install/setup.bash
ros2 run zzx_http_contracts competition_http_fake --kind arm --scenario unprepared
```

终端会打印实际 URL，如 `http://127.0.0.1:随机端口`。Ctrl+C 关闭。另开终端把输出的端口代入 curl 查询 `/api/status`，不要沿用现场地址。
可用 `--kind hand` 启动手部假服务，或 `--port 18087` 指定空闲端口；端口占用会报错，不会结束已有程序。

建议先读 test_plan_success_execute_failure_then_prepare_success，再读 Model.handle，最后对照 LoopbackClient.prepare。这样能看到“现场现象 → 服务模型 → 客户端处理 → 测试证据”的对应关系。

## 回归与边界

```bash
colcon test --packages-select zzx_http_contracts zzx_task_runtime zzx_execution
colcon test-result --test-result-base build/zzx_http_contracts --verbose
colcon test-result --test-result-base build/zzx_task_runtime --verbose
colcon test-result --test-result-base build/zzx_execution --verbose
```

覆盖独立的关节顺序期望值、完整 prepare 请求顺序、误标签实际反馈核对、200/400 失败解析、延迟、断连、取消与状态变化、手指保留更新、非法输入、损坏响应及已安装 CLI 启动。
这些是已知协议的回归证据，不是完整厂商兼容认证，不能证明真实服务始终具有相同行为。

2026-10-02 验收：新增 HTTP 包 17 项测试通过；持久化包 10 项通过；执行包汇总 41 项通过（含测试容器记录），均无错误、失败或跳过。已测试安装后的假服务命令启动、环回查询与关闭；未连接真机，未验收 VS Code 图形界面。

B10 下一步将实现独立 HTTP ROS 2 后端节点，隔离网络阻塞与 Action 回调、建立能力声明，并将取消/失联与 B07/B08 的状态语义对接。现阶段没有新增可连接现场真机的驱动入口。
