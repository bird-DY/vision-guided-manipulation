# B10：HTTP 与 ROS 2 Action 的适配边界

## 本阶段完成什么

新增 `ros2_ws/src/zzx_http_backend`，将 B09 已验证的 HTTP 协议接入：

- `/zzx/manipulation/move_arm`：七轴左臂命名关节目标。
- `/zzx/manipulation/control_hand`：O10 命名稀疏位置目标。

当前部署**仅允许环回假服务**。现场 profile 的 `automatic_motion_allowed=false` 没有被修改；适配器拒绝 192.168.0.22 等非环回地址。真机入口、生命周期和跨进程所有权尚未验收，不能把本阶段写成真机接入完成。

## 代码怎么读

| 文件 | 职责 |
| --- | --- |
| `zzx_http_backend/node.py` | Action 接收、单资源互斥、取消标记、反馈、终态、能力发布 |
| `zzx_http_backend/worker.py` | 工作线程内的 HTTP 准备、命令发送、反馈核对、停止请求 |
| `config/capabilities.json` | 已核验协议能力的本地声明，启动读取 |
| `test/test_actions.py` | 真实 DDS → Python 适配器 → HTTP socket → 假服务的集成测试 |
| `test/test_worker.py` | 无网络的取消顺序、旧反馈和输入校验测试 |

客户端复用 B09 的严格协议实现；B09 源自现场 ArmHttpClient/HandHttpClient 已确认的字段和行为，没有导入整个比赛算法工程。
这里采用 Python 是为了复用 HTTP 协议处理；已有 C++ MoveArm 假后端及 B07 监控模块保留，没有改回 Python。
HTTP 适配器采用相同的“命令响应 + 状态核对 + 稳定窗口”原则，但目前**没有直接调用 C++ ExecutionMonitor**；不要把两个实现视为完全等价。未来引入真控制器时需统一反馈时钟和监控适配。

## 为什么网络不会占住 ROS 回调

```text
Action Goal → 校验/占用 arm 或 hand → 提交线程池任务
  → async execute 等待 ROS Future（不是同步 future.result 阻塞）
  → 定时器读取已完成的工作任务 → 生成 Action 结果
```

HTTP 工作在线程池；ROS executor 有 4 个线程，Action 使用可重入回调组。共享状态以锁保护。
取消回调只设置事件，并为机械臂提交独立的尽力取消请求，不等待网络。主工作任务仍必须确认原命令结果和停止状态。
`~/get_status` 是 Trigger 服务，只读取本地 busy/latched 状态，不访问 HTTP。可用它验证长 HTTP 请求期间回调仍响应。
每个设备最多一个动作；arm 与 hand 各有占用，不代表已经完成全系统碰撞协调或跨进程互斥。

## 能力声明与拒绝规则

启动读取本地 capabilities JSON，并以 transient-local QoS 发布：

- `~/arm_capabilities`
- `~/hand_capabilities`

现场没有已验证的 HTTP capabilities 端点，因此不虚构远程能力查询。声明来自经过回归的协议子集。

机械臂：支持 joint_target、plan_only、prepare 和 cancel 请求；不支持 pose、trajectory、force_control。设置 `motion_mode:=trajectory` 在启动时失败，不降级成其他运动模式。
灵巧手：只支持归一化位置；必须 `speed_scaling=1`、`max_effort=0`、`stop_on_contact=false`。协议没有可靠映射时，非默认速度、力控、接触停止和弧度模式明确返回 UNSUPPORTED。

这里 speed=1 表示不施加未支持的速度参数，不代表设备按某个已知物理速度运行。需要比赛中的分步柔和夹取时，应另做有状态插值与中断测试，不能直接宣称本节点已经具备。

## 执行与取消语义

### 机械臂

每次实际执行前：退出示教 → pos_vel → enable → 核对左臂电机反馈。plan_only 跳过准备，不使能电机。
按名称转换为现场固定七轴数组顺序，发送一次 joints 命令；成功响应后查询反馈，必须 moving=false、位置误差不超过 0.001 rad、估算速度不超过 0.01 rad/s，持续稳定窗口后才返回成功。
反馈时间戳必须递增；单次状态读取或样本间隔超过 0.2 秒不能继续累计稳定证据。

取消事件会阻止后续准备命令。动作已经发送时，先尽力请求 cancel，原请求结束后再次取消并确认稳定停止。取消响应本身不是停止证明。
若原 POST 超时或断连，不能排除它稍后才被服务端执行，因此即使 cancel 返回成功也不宣称确认停止：返回 RESULT_UNKNOWN 并锁定新动作。会尽力请求停止，但不自动重发运动。

HTTP 服务端的 source timestamp 仍需在真机上核验。当前只检查递增、读取时长和采样间隔，没有跨主机时钟同步，也不能识别所有“返回时间在更新而底层数据陈旧”的情况。

### 灵巧手

先读取十维 position，覆盖目标中指定的关节，再发送完整数组；回读目标连续匹配后返回成功。final_positions 顺序与请求 joint_names 一致。
这只是位置回读核对，没有接触、力矩或真实停止证据。
hand 没有已验证的取消 API。发送后收到取消请求，返回 STOP_UNCONFIRMED 并锁定，不擅自张开或握紧作为“停止”。

### 结果未知与退出

断线、结果未知或停止不确定会将对应设备 latched=true，后续 Goal 被拒绝。当前没有解锁服务；假服务调试需核对后重启，真机不能照搬为恢复策略。
可与 B08 入口连接，RESULT_UNKNOWN 会保留 NEEDS_RECONCILIATION。适配器自身没有数据库，绕过 B08 的直接请求也没有持久化保护。
进程退出期间不保证客户端收到终态；必须按未知结果处理，不能因为窗口关闭就判断物理动作停止。

## 在 VS Code / WSL 中启动

每个终端先加载同一 ROS 环境：

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to zzx_http_backend
source install/setup.bash
export ROS_DOMAIN_ID=90
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

构建只需执行一次。三个终端分别运行：

```bash
# 终端一
ros2 run zzx_http_contracts competition_http_fake --kind arm --port 18087
# 终端二
ros2 run zzx_http_contracts competition_http_fake --kind hand --port 18088
# 终端三
ros2 run zzx_http_backend http_action_backend --ros-args -r __ns:=/b10
```

不要让 C++ 假 Action 服务与本适配器占用相同 Action 名称；示例采用 /b10 命名空间。端口被占用时换空闲端口，并同步修改 arm_url、hand_url 参数，不结束无关程序。

第四个终端可先查看本地状态，再发送食指测试：

```bash
ros2 service call /b10/http_action_backend/get_status std_srvs/srv/Trigger '{}'
ros2 action send_goal /b10/zzx/manipulation/control_hand zzx_interfaces/action/ControlHand \
  '{position_unit: 1, joint_names: [index_pip], positions: [0.0], speed_scaling: 1.0, max_effort: 0.0, stop_on_contact: false, timeout: {sec: 5}}' --feedback
```

启动节点本身不发 HTTP、不使能、不运动。默认 URL 仅为 127.0.0.1:18087 / 18088；所有参数仅启动读取。
请求超时默认 1 秒、稳定窗口 0.1 秒、停止确认期限 1 秒，可通过 request_timeout_seconds、settling_seconds、stop_timeout_seconds 调整。停止或尽力取消可能让最终结果晚于动作原始 timeout，客户端等待预算需留出余量。

## 测试与下一步

```bash
colcon test --packages-select zzx_http_backend zzx_http_contracts zzx_task_runtime zzx_execution
colcon test-result --test-result-base build/zzx_http_backend --verbose
```

集成测试验证：prepare 与命名顺序、仅规划不使能、手指保留更新、力控拒绝、执行失败后的停止确认、断线锁定、阻塞期间状态查询/取消、轨迹模式启动拒绝。
工作任务测试补充：准备中取消后不再变更设备、手部取消不可伪报停止、旧反馈拒绝、非法输入拒绝、真机地址拒绝。

2026-10-02 全量相关回归：HTTP 适配包 12 项、HTTP 契约包 17 项、持久化包 10 项、执行包汇总 41 项（含测试容器记录），全部通过。仅在 WSL、本地 DDS 与 HTTP 假服务测试，未验收真机或 VS Code 图形界面。

B11 下一步是 bringup、生命周期和单命令所有权，进一步约束节点组合、运行状态与启动/停止顺序。之后才讨论受控真机接入。
