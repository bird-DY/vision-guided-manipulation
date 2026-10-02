# B08：任务幂等、结果持久化与未知结果核对

## 1. 解决什么问题

比赛时客户端超时、网络断开或程序退出，都不能证明机械臂没有动作。直接重试可能重复抓取、重复按按钮。

本阶段新增 `zzx_task_runtime` Python 包，用 SQLite 管理请求身份和结果，再调用已有 C++ MoveArm。SQLite、hashlib、fcntl 都来自 Python 标准库，无需数据库服务器或新 VS Code 插件。

这是**关节动作的持久化入口**，不是完整比赛任务、视觉闭环或 ExecuteTask Action server。task_id 在这里标识一次关节操作。后续任务编排可以复用 Ledger，但多步骤任务仍需独立步骤 ID 和恢复规则。

## 2. 模块职责

| 文件 | 职责 |
| --- | --- |
| `zzx_task_runtime/ledger.py` | 请求摘要、SQLite 事务、线程去重、进程所有权、重启恢复、人工核对审计 |
| `zzx_task_runtime/move_arm.py` | 参数规范化、持久化 operation_id 到 ROS Goal UUID 的映射、结果一致性核验 |
| `zzx_task_runtime/cli.py` | run / show / reconcile 命令行入口 |
| `test/test_ledger.py` | 并发、冲突、存储失败、SIGKILL 与审计回归 |
| `test/test_ros_dispatch.py` | 20 个并发请求经真实 DDS 调用 C++ 假服务的集成验收 |

以上文件均位于 `ros2_ws/src/zzx_task_runtime`。C++ 继续负责执行与反馈监控，Python 负责低频编排与持久化，不在运动控制周期里执行数据库写入。

## 3. 请求流程与状态

```text
JSON 输入 → 校验/按关节名排序 → SHA-256 摘要
  → SQLite 写入 task_id / payload_hash / operation_id / DISPATCHING
  → 事务提交成功
  → 使用该 operation_id 作为 ROS Goal UUID，发送一次 MoveArm
  → 保存完整后端结果或 NEEDS_RECONCILIATION
```

同 ID 同参数：返回原记录，不再发送。同 ID 不同参数：Conflict，不能覆盖旧任务。
关节顺序不同但名称与位置对应相同，规范化后视为同参数。Action 路径、缩放、plan_only、动作超时都属于摘要内容；客户端等待结果的时限不属于动作参数。

| 状态 | 含义 | 自动重发 |
| --- | --- | --- |
| DISPATCHING | 意图已落盘，发送/结果处理正在进行 | 禁止 |
| SUCCEEDED | 已知成功，或有审计记录的人工核对成功 | 禁止 |
| FAILED / CANCELED | 已知终态 | 禁止 |
| NEEDS_RECONCILIATION | 超时、异常、停止不确定或上次进程中断 | 禁止 |

重启时持有数据库进程锁，将遗留 DISPATCHING 改为 NEEDS_RECONCILIATION。
即使进程恰好在“写入意图之后、实际发送之前”退出，也保守地要求核对，不能猜测没有发送。

持久化失败发生在发送前：不发送。失败发生在发送后：保留未决记录，恢复时核对。
通信超时不是运动失败证明；C++ 返回 TIMEOUT、STOP_UNCONFIRMED、RESULT_UNKNOWN，也保守进入核对状态。

## 4. 并发与保证的边界

- 一个 Ledger 支持多线程提交；20 个同 ID 请求只有一个拿到发送权，其余可能先看到 DISPATCHING，之后通过 get 查询最终结果。
- 不同 ID 在已有 DISPATCHING 或 NEEDS_RECONCILIATION 时被拒绝，防止未决动作后继续下发。
- flock 确保**同一个本地数据库文件**同时只有一个进程拥有调度权。第二个 CLI 进程会报忙，不会加入已运行进程等待结果。
- SQLite 使用 WAL 与 synchronous=FULL，意图先于物理请求提交。
- 这提供本入口内的保守“至多一次发送”，不承诺分布式 exactly-once。SQLite 与机器人控制器之间不存在原子事务。
- 删除数据库、换数据库路径、绕过入口直接调用 MoveArm，都不受这层保护。同一设备必须固定使用同一个数据库。
- 数据库放 WSL 的 Linux 文件系统，例如 `~/.local/state/zzxrobot/`；不要用 `/mnt/d` 或网络共享路径承载运行数据库。备份时需要一致性备份，不能只复制正在写入的主文件而忽略 WAL。
- 尚无后台网络网关、跨主机所有权、自动后端结果查询、认证授权或多步骤任务恢复。

## 5. 在 VS Code 中演示

在 WSL 打开 `~/vision_guided_manipulation`，两个终端均先执行：

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to zzx_task_runtime zzx_execution
source install/setup.bash
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

终端一只启动假后端：

```bash
ros2 run zzx_execution move_arm_server --ros-args -r __ns:=/b08_fake
```

终端二发送操作：

```bash
ros2 run zzx_task_runtime durable_move_arm run \
  --task-id demo-joints-001 \
  --request src/zzx_task_runtime/examples/joint_request.json
```

再次执行同一命令，返回持久化结果而不运动。修改参数但仍使用相同 ID，会报冲突。
需要真正执行一个新动作时才换新 ID；不要为了绕过未知状态而换 ID 或删除数据库。

查看已记录状态（另一个持久化 CLI 仍运行时会因所有权锁而拒绝）：

```bash
ros2 run zzx_task_runtime durable_move_arm show --task-id demo-joints-001
```

新操作使用 `--wait-seconds 0.1` 可测试等待结果超时；这**不会自动取消动作**，后端可能仍在运动，因此记录未知并阻止后续请求。
结果里包含 task_id、摘要、operation_id、完整后端结果、更新时间和人工核对审计。操作 ID 与 C++ 返回的 execution.operation_id 必须一致。

CLI 对成功状态返回退出码 0，其他状态/错误返回 2；非零退出码不表示物理上“没有执行”。

## 6. 人工核对不是重试按钮

先查询控制器活动目标、关节反馈及现场状态；必要时按现场停止规程处理。仅验证 position 接近目标不足以证明整个任务成功。
确认结果后使用 reconcile，必须写明证据，例如在假后端演示中：

```bash
ros2 run zzx_task_runtime durable_move_arm reconcile \
  --task-id demo-joints-002 --state FAILED \
  --evidence '已核对假后端目标及关节反馈，确认没有活动动作；本次操作未完成'
```

只有 NEEDS_RECONCILIATION 能被核对。核对只改变记录，不发送运动；原后端证据保留在 backend_result，人工结论追加到 reconciliations。
这依赖操作人的真实核对，不是自动硬件证据验证；不要盲目粘贴示例来解锁。

## 7. 回归测试与下一步

```bash
colcon test --packages-select zzx_task_runtime zzx_execution
colcon test-result --test-result-base build/zzx_task_runtime --verbose
colcon test-result --test-result-base build/zzx_execution --verbose
```

覆盖 20 线程并发去重、相同 ID 参数冲突、持久化重读、写入拒绝、未知结果阻塞、核对审计、进程 SIGKILL 后恢复，以及真实 ROS 假服务的一次发送验证。
SIGKILL 测试在独立子进程模拟“物理请求已发送、结果未写入”的中断；真实 ROS 并发与结果超时在另外的集成测试验收，不是实际机器人断电试验。

2026-10-02 验收：连续两轮回归均通过。持久化包 10 项测试全部通过；既有执行包汇总 41 项、0 错误、0 失败、0 跳过（含测试容器记录）。已安装的 durable_move_arm 命令入口验证通过。没有连接真机，也未验收 VS Code 图形界面。

下一阶段 B09 将建立比赛 HTTP 假服务及契约测试，再逐步把现场 HTTP 行为与这一层结果语义接起来，不直接连接真机验证。
