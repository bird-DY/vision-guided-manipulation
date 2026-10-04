# B13：单实例相机入口

## 目标与真实状态

解决现场 Viewer、算法服务和预览脚本争抢同一 USB 相机的问题。只有采集入口打开设备，预览、算法和记录器订阅 ROS 2 话题。原生 OrbbecViewer 仍然直接打开 USB，不能与本入口同时使用；这里的共享预览使用 `rqt_image_view`。

当前完成软件入口与合成/模拟契约验收，**尚未完成 Gemini335 真机验收**。开发 WSL 未发现 `/dev/video*`，未安装 `orbbec_camera` 或 `pyorbbecsdk`；单凭没有视频节点也不能断言所有 USB 设备不存在。没有修改 USB、固件、系统网络参数或连接机械臂。

| backend | 用途 | 验证状态 |
| --- | --- | --- |
| `synthetic` | 默认，合成 BGR/米制深度/CameraInfo，无 USB 访问 | 真实 ROS 跨进程、录制和故障测试 |
| `orbbec` | 首选硬件路径，受管启动官方 Gemini 330 系列 ROS wrapper | 参数与启动适配已实现；驱动未安装，USB/固件/实际话题待验收 |
| `sdk` | 可选 SDK 兼容路径，单采集线程加一帧队列 | SDK 契约模拟通过；真实 SDK/设备未验收 |

硬件路径必须显式设置 `allow_device:=true`，缺依赖时报错退出，**不降级成合成成功**。本阶段任何模式都报告 `motion_eligible=false`，流就绪不等于标定有效或可以进行机械臂运动。

## 数据链路

```text
Gemini335 -> 官方 ROS driver -> 受管相机入口 -> 统一图像话题
       或 -> 单实例 SDK worker -> 相机入口 -> 统一图像话题
                                             |-- rqt_image_view
                                             |-- 算法订阅者
                                             |-- rosbag2 recorder
```

官方驱动的原始话题在 `/zzx_camera_raw`，业务侧只订阅 `/camera`。入口不做缩放、伪造 D2C 或运动控制。SDK 使用 `AlignFilter` 对齐到彩色流；失败、尺寸不符、不支持的畸变模型都停止，不回退到 resize。SDK 的逐帧深度比例先从毫米换算到米，输出 `32FC1`；官方 ROS 分支保留原始合法编码及头信息。

| 话题 | 类型 | 单位/说明 |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | `bgr8` 或 `rgb8` |
| `/camera/depth_registered/image_raw` | `sensor_msgs/Image` | `16UC1` 按毫米，`32FC1` 按米；硬件尺度仍需 B15 验证 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 对应彩色尺寸及光学坐标系 |
| `/camera/depth_registered/camera_info` | `sensor_msgs/CameraInfo` | 必须与彩色内参及光学坐标系一致 |
| `/camera/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 状态、原因、会话 ID、配置 hash、流模式、标定状态 |

状态查询：`/camera/camera_manager/get_status`，`std_srvs/srv/Trigger`，message 为 JSON。

输出图像/CameraInfo 使用 Reliable、Volatile、KeepLast(5)。Reliable 不保证及时性，慢订阅者可能形成反压；B14 还必须做年龄、同步和队列门控。官方原始话题显式请求 `default` QoS；实际驱动版本的 QoS 映射要在硬件验收时核对。

## 状态与故障

- `STARTING`：等待 color/depth 和两路 CameraInfo；默认启动期限 15 秒。
- `STREAMING`：四路消息已通过尺寸、编码、缓冲区长度、光学 frame 和内参一致性检查，且最近持续到达。
- `ERROR`：任一路超时、流签名变化、内参变化、重复发布者或驱动退出后锁定；停止转发，终止受管驱动/SDK 采集。
- 恢复连接不会自动解除锁定，必须停止并重新启动。新进程产生新的 `stream_session_id`，标定仍为 `unvalidated`；故障后为 `invalidated`。

参数只读：width/height/fps/backend 等只能重启修改。不要绕过入口直接调用官方驱动的动态配置服务；本版不保证检测到所有不改变图像签名的外部参数变更，例如单独修改曝光或 FPS。模式管理是合作式约束，不是安全认证。

SDK 当前用主机发布时刻填写 ROS 头，不能宣称是传感器采集时刻；官方分支要求 vendor system 时间并原样转发。跨模态时间差、采集时间映射、有效深度比、TF 和精度门控归 B14/B15，当前不补造同步保证。

`~/.local/state/zzxrobot/owners/camera_ingress.lock` 的文件锁跨 backend、跨 ROS namespace 互斥。锁只覆盖同用户、同机器、遵守入口的进程；Windows Viewer、其他用户/主机和直接调用 SDK 不受保护。不要删除锁文件来“解锁”，正常退出后 OS 释放锁。当前设计有意只支持一台受管相机。

相机状态尚未接入任务行为树的运动许可；相机失联只关闭图像入口，不能把它理解为机械臂急停。

## WSL 中启动与共享

先构建：

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to zzx_bringup
```

**每个新终端**都执行：

```bash
cd ~/vision_guided_manipulation
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source scripts/camera_env.sh
```

脚本仅为当前终端设置 CycloneDDS、本机通信和数据报配置。相机 launch 对没有显式 DDS 配置的子进程使用同一份默认配置；外部订阅者仍需上述环境。已有自定义 `CYCLONEDDS_URI` 不会被 launch 覆盖，但 source 此脚本会替换当前终端的值。

终端一，安全的合成演示：

```bash
ros2 launch zzx_camera camera.launch.py backend:=synthetic fps:=5
```

也可以在统一启动中启用相机，不要与上一条同时运行：

```bash
ros2 launch zzx_bringup stack.launch.py profile:=fake camera_backend:=synthetic
```

终端二，查看状态或预览：

```bash
ros2 service call /camera/camera_manager/get_status std_srvs/srv/Trigger '{}'
ros2 run rqt_image_view rqt_image_view
```

在 rqt 中选择 `/camera/color/image_raw`；需要时另开实例查看深度。GUI 预览本轮未实测，已核对工具安装。算法节点订阅同一组话题，不再创建自己的 SDK Pipeline。

终端三，记录五路消息：

```bash
mkdir -p outputs
ros2 bag record --qos-profile-overrides-path \
  ros2_ws/src/zzx_camera/config/record_qos.yaml \
  -o outputs/camera_demo \
  /camera/color/image_raw /camera/depth_registered/image_raw \
  /camera/color/camera_info /camera/depth_registered/camera_info /camera/diagnostics
```

结束录制用该终端 Ctrl+C，然后 `ros2 bag info outputs/camera_demo` 检查各话题计数。再次录制应使用新目录名。`outputs/` 已被 Git 忽略。不要仅凭“Subscribed to topic”认定数据已经录入。

## 本轮发现的 WSL 传输问题

小型彩色帧能够跨进程录制，但较大彩色和深度帧计数为零，同进程深度订阅正常。显式 BestEffort 和单独切换 Reliable 均不能解决；限定 `MaxMessageSize=1200B`、`FragmentSize=1024B` 后恢复。由这些对照可以定位到当前环境的大消息/数据报路径，但没有抓包证明具体内核或网络根因，不写成已确认的驱动缺陷。

配置在 `config/cyclonedds_wsl.xml`，无需 sudo/sysctl，也未更改用户全局 shell。验证包含 32×16@20Hz 和 1280×720@5Hz 合成 RGB-D，同时有算法订阅者及独立 rosbag 进程，五路消息均有记录。没有证明 1280×720@30Hz、跨机器网络或真实 USB 吞吐达标。小数据报增加包数量，部署到其他网络时需重新压测。

## 硬件接入前

参考版本记录在 `config/vendor_sources.json`，是本次核对的上游 commit，不是已安装版本或运行时强制锁：

- Orbbec ROS wrapper：`296156521b60bdf813cb94c46cbb55562bbdc453`。
- pyorbbecsdk：`0f089c9bf6e618e7782228ee6bcf666e3339ae78`。

优先按[官方 ROS wrapper 安装说明](https://github.com/orbbec/OrbbecSDK_ROS2)在独立 `~/orbbec_ws` 安装对应版本，source 该 overlay，再 source 本项目。不要将整份上游 SDK、二进制或驱动构建产物提交进本仓库。[官方配准说明](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/configuration/align_depth_color.html)使用 Gemini 330 系列 launch 和 `depth_registration` 参数，本入口固定开启 SW D2C 并关闭原生 Viewer、点云及网络设备枚举。

SDK 兼容后端参照[官方 Python API](https://github.com/orbbec/pyorbbecsdk/blob/v2-main/stubs/pyorbbecsdk.pyi)，要求当前 ROS Python 能导入 SDK、恰有一台可见 USB 相机，支持显式 RGB/Y16 模式及 Brown 畸变模型。没有尝试猜测或降级到未知格式。

完成 WSL USB 转发、驱动安装、设备型号/固件和权限核对，关闭 Windows Viewer 与旧 Python 相机服务后，才手动执行其中一个：

```bash
ros2 launch zzx_camera camera.launch.py backend:=orbbec allow_device:=true
# 兼容路径，不能与上一个并行：
ros2 launch zzx_camera camera.launch.py backend:=sdk allow_device:=true
```

首先验证编码/单位/有效深度、D2C 叠加、实际分辨率/FPS、内参、拔插行为和资源释放；未通过前不能将流接入机械臂视觉定位。

## 代码阅读顺序与验收

1. `guard.py`：文件锁、四路流的契约检查和锁定状态。
2. `sources.py`：合成/SDK 数据转换、单线程采集、一帧队列、调用 SDK 对齐过滤器，绝不 resize。
3. `node.py`：统一话题、子进程生命周期、诊断、只读参数。
4. `launch/camera.launch.py`：默认合成和硬件显式开关；`zzx_bringup/launch/stack.launch.py` 增加可选 `camera_backend`，默认 `none`，不改变旧启动行为。
5. `test/`：非法帧、标定失效、两订阅者、重复入口、断流、新会话、驱动子进程清理、SDK 模拟、真实 rosbag 录制。

```bash
cd ~/vision_guided_manipulation/ros2_ws
colcon test --packages-select zzx_camera zzx_bringup zzx_datasets zzx_http_backend
colcon test-result --test-result-base build/zzx_camera --verbose
```

2026-10-04 验证记录：8 个依赖及应用包构建通过；相机 34 项、bringup 6 项、数据工具 29 项、HTTP 适配 16 项测试全部通过，共 85 项，无错误、失败或跳过。包含独立 rosbag 进程实际写入五路消息，并非仅检验话题发现。联合启动测试还覆盖终端 SIGINT 与 launch 转发中断叠加的退出清理：相机在清理阶段忽略后续 SIGINT，两个子进程干净退出，锁均释放。

下一步 B14 增加 RGB-D 同步、图像年龄与质量门控；B13 真机驱动安装和 USB 验收仍单独保留，不能用合成测试替代。
