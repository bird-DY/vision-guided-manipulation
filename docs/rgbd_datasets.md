# B12：RGB-D 数据清单与离线回放

## 本阶段解决什么

现场图片需要成为可追溯的回归数据，而不是混在截图、叠加图和日志里的若干文件。新增 `zzx_datasets`，负责导入、哈希校验、批次隔离、读取原始数组和生成确定性统计。代码不依赖 ROS 节点，不发布话题、不访问相机或机械臂，也不执行图像识别。

这里的“回放”是按样本 ID 稳定排序，逐帧解码原图/深度并输出 JSONL；不是按传感器时间播放的 rosbag，不产生 `/clock` 或 TF。`load_frame(root, sample)` 可供后续离线算法读取原始 BGR 与深度数组。时间同步和 ROS 图像话题在 B13/B14 实现。

## 实际数据覆盖

2026-10-03 从 Windows `D:\Desktop\zhongkong\final\samples` 只读导入：

| 场景 | 样本数 | 深度 | 人工标签 |
| --- | ---: | --- | --- |
| 面板灯 | 27 | 无配套深度 | 未确认 |
| ChArUco | 13 | 13 组 NPY | 未确认 |
| 相机检查 | 2 | 2 组 NPY | 未确认 |
| 编号块 | 0 | 缺失 | 缺失 |
| 几何物体 | 0 | 缺失 | 缺失 |

共 42 个样本，15 组含深度。ChArUco 文件缺 `intr_010`、`intr_012`，没有补造样本。检测到 17 个 Marker、24 个角点的旧统计只能证明当时检测器给出的结果，不是人工真值，也不能证明标定精度。

原图、NPY、日志不复制到 Git，只提交 `data/manifests/competition_final.json` 的文件清单与 SHA-256。使用其他机器时需要另行取得原始数据并通过 `--root` 指定位置；清单本身不能还原原图。分享原图之前应审核现场人物和其他隐私。

9 个未映射文件记录在清单 `unmapped_files`，包含机械臂调试、运行日志、单独叠加图及 `intrinsics_result.json`。它们没有被当成独立图像样本。旧内参结果尚未在 B15 验证，不能作为已批准标定。

## 不可补造的历史信息

- 旧保存代码写入 `frame.depth_m`，本专用导入器仅接受浮点二维深度，记 `depth_scale_m=1.0`；通用清单支持整数深度，但必须显式声明米制比例。
- JSON 中的 `timestamp` 来自主机 `time.time()`。导入为 `host_timestamp_ns`，不填写未知的彩色/深度传感器时间。纳秒字段只是表示单位，不代表纳秒测量精度。
- 面板文件名中的时间不提升为传感器时间；相关时间保持 `null`。
- 当前配置不能证明历史采集配置，所以 `capture_config=null`。内参保留历史 JSON 数值，但 `calibration_id=null`，不声称已验证。
- 旧 `orbbec_gemini335.py` 在尺寸不一致时用最近邻缩放深度。缩放不等于 D2C 配准；全部历史样本保持 `alignment=unknown`。
- 没有可核实的独立采集批次边界，保守地把全部数据归入 `competition_final_unseparated_session`，`split=unassigned`。不能为了凑训练/测试集而随机拆同一采集序列。

本次审阅的当前源代码指纹（不是当年运行版本证明）：

```text
final_app/camera/orbbec_gemini335.py
ed78ee780f70d41e0155b9acb5cc59850151aebb7f77c2886e9ccf0b008c7b35
final_app/api/server.py
258916cb587ea8c106fd1ea8b10571b4c50f9c016efce6d5ddf585b249f05dbb
```

## 代码导读

| 文件 | 作用 |
| --- | --- |
| `ros2_ws/src/zzx_datasets/zzx_datasets/manifest.py` | 数据契约、哈希、路径边界、尺寸/内参检查、批次和内容泄漏检查；`load_frame` 读取原数组，不缩放 |
| `ros2_ws/src/zzx_datasets/zzx_datasets/cli.py` | `inventory` 导入历史格式；`replay` 逐帧统计；命令行入口 |
| `ros2_ws/src/zzx_datasets/test/test_dataset.py` | 临时合成小图和深度测试，不依赖现场文件 |
| `data/manifests/competition_final.json` | 真实现场资产索引，可审查 diff，不含原图 |

`import-final` 是特定历史格式适配器，不是通用相机录制器。它匹配递归 `*_color.jpg`，以及根目录 `*_panel*.jpg`；不会把叠加图当原图。导入不会覆盖已有清单，避免丢失后续人工标签。

## 清单契约

顶层 `schema_version=1`、`dataset_id`、非空 `samples`。每个样本记录：

- `id`：唯一样本名；`batch_id`：真实采集序列分组；`source`：来源；`scene`：场景类型。
- `split`：`train/tuning/holdout/unassigned`。同批次不可跨分区，彩色或深度内容完全相同也不可跨分区。近重复不同哈希由批次治理解决，本版不做感知哈希。
- `assets`：至少有 color，可附 depth/metadata/overlay；每项含相对 POSIX 路径、字节数、SHA-256。拒绝路径越界、指向根目录外的符号链接、缺失和篡改。
- `color_size/depth_size`：`[width,height]`；尺寸和解码数据一致。内参存在时必须匹配彩色分辨率，焦距为正有限数。
- `depth_scale_m`：原始深度数值乘此系数得到米；无深度时为 `null`。
- `color_timestamp_ns/depth_timestamp_ns/host_timestamp_ns`：正整数或 `null`，不得混用。
- `intrinsics/capture_config/calibration_id`：历史不可核实信息保留未知。新采集配置应记录流模式、曝光、增益、白平衡、序列时钟和对齐方法。
- `alignment`：`unknown/unaligned/registered_to_color`；声明配准还必须提供非空 `alignment_evidence` 和 `calibration_id`，且彩色/深度尺寸一致。文字证据仍需 B15 人工/几何复核。
- `label.status`：`unknown/verified`。未知标签的 `value` 必须为空；已确认标签必须有 `value` 与 `reviewer`。`verified` 只是人工标注声明，不是算法正确率。

统计明确输出 `evaluated_samples=0`、`success_rate=null`。即使已有确认标签，本工具也没有预测结果，不计算成功率。后续 B18 按保留集生成混淆矩阵时再统计。

## 在 WSL 中执行

```bash
cd ~/vision_guided_manipulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select zzx_datasets
source install/setup.bash
cd ..

ros2 run zzx_datasets rgbd_dataset validate \
  data/manifests/competition_final.json \
  --root /mnt/d/Desktop/zhongkong/final/samples

mkdir -p outputs/b12
ros2 run zzx_datasets rgbd_dataset replay \
  data/manifests/competition_final.json \
  --root /mnt/d/Desktop/zhongkong/final/samples \
  --min-depth-m 0.1 --max-depth-m 2.0 > outputs/b12/replay.jsonl
```

回放先校验整份清单，再输出帧；筛选某个 split 也不会跳过其他分区的泄漏检查。每行含样本 ID、标签状态、来源时间、有效深度比例及 P05/中位数/P95。有效范围为 `(min,max]`，过滤零、非有限数和超范围值。默认 0.1～2 米只是统计窗口，不是相机量程或标定结论；65.535 米等异常远值不会污染此窗口的中位数。全无效返回 `median_m=null`。全部帧 `motion_eligible=false`。

有新数据时创建另一份清单，不覆盖已审核数据：

```bash
ros2 run zzx_datasets rgbd_dataset import-final \
  --root /path/to/new/final_samples \
  --batch-id independent_capture_002 \
  --output data/manifests/new_capture.json
```

`--output` 应放在原始数据目录之外。在 VS Code 中用 WSL 打开项目，先读 `manifest.py` 的 `validate`，再读 `cli.py` 的 `replay`；不需要新增编辑器插件。

## 数据补齐顺序

1. 独立采集面板红/绿/白/熄灭/反光序列，保存完整曝光和时间信息；人工核对标签，尤其复现“白红去按绿”的误判。
2. 新采集编号块和几何物体原始 RGB-D，不能用手机任务区域照片代替深度。
3. 根据可追溯的采集记录确定批次，整个批次分配到训练、调参或保留测试；保留测试不得参与阈值选择。现有历史集合可整体作为调参集，不能从中声称独立测试成绩。
4. B13/B14 提供单实例相机、同步和元数据；B15 验证内参、深度单位、D2C；B16 才做手眼/TCP。当前清单不允许直接推导机械臂目标。

## 验证记录

2026-10-03：新增包构建成功，29 项测试全部通过，无错误、失败、跳过；真实 42 个样本校验通过并输出 42 行 JSONL。覆盖路径越界、文件缺失/篡改、批次/重复内容泄漏、元数据非法、无效深度、RGB-only、确定性顺序、禁止隐式缩放/配准和输出防覆盖。

```bash
cd ~/vision_guided_manipulation/ros2_ws
colcon test --packages-select zzx_datasets
colcon test-result --test-result-base build/zzx_datasets --verbose
```

B12 工具和现有资产整理已完成；独立保留集、人工真值、缺失场景以及历史配准/同步仍待补齐。没有相机在线、视觉精度、真机或运动安全验收。
