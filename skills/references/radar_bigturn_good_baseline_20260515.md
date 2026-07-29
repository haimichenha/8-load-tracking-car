# 2026-05-15 雷达大转弯回原点稳定版记录（历史只读基线）

> **当前项目边界（2026-07-26）**：本文件仅保留树莓派/雷达侧的历史算法、部署和日志经验；不定义 STM32F103ZET6 的 UART、TIM、GPIO、PWM、编码器或电机接线。当前硬件 pin-map 必须以 `rules.md` 为准。

## 1. 记录目的

本文件记录 2026-05-15 最近一轮对树莓派雷达 / Cartographer 有效的修改思路、具体修改点、涉及函数、验证结果和回退路径。

当前结论：雷达侧已经从“静止/大转弯后 10~30cm 级漂移不可接受”优化到“大转弯一圈返回后误差很小、可进入下一阶段”的状态。后续不要再大幅修改雷达参数，除非先复制当前基线并保留回退路径。

## 2. 当前雷达侧正式基线

本地当前只读指针：

```text
logs/rpi_lidar_good_min_score_078_20260515_143748_READONLY/
logs/CURRENT_LIDAR_BASELINE.txt
```

本地正式包：

```text
logs/rpi_lidar_good_min_score_078_20260515_143748.tar.gz
```

SHA256：

```text
0df93bb6beb69556b14fc8216c08070931923124121328918abc6fbb27c50629
```

树莓派正式包：

```text
/home/ubuntu/codex_lidar_watch/snapshots/good_bigturn_micro_20260515_041241.tar.gz
```

## 3. 有效修改思路

### 3.1 先解决近点污染

早期日志中原始点云出现过 `0.15m ~ 0.18m` 近点。结合小车底座、线材、手、边缘遮挡等因素，近点容易把 scan matching 拉偏。

有效策略：

```lua
TRAJECTORY_BUILDER_2D.min_range = 0.8
```

含义：小于 `0.8m` 的点不参与 Cartographer 2D 匹配/建图。

### 3.2 保留足够侧向/后向特征，避免大转弯后匹配丢约束

先前后方过滤较狠：`front_half=120deg, rear_min=1.15m, rear_keep_every=3`。大转弯/180° 后，侧后方特征会变成下一段行驶的重要环境参考，过滤过狠会让转头后 x/y 位姿更容易漂。

当前有效策略：

```text
front_half=150deg
front_min=0.80m
rear_min=1.00m
rear_keep_every=2
```

效果：大转弯后可用环境特征更多，回原点误差明显减小。

### 3.3 收紧局部匹配的平移自由度，不让“大转头”被估计成“大平移”

当前有效 Cartographer 局部匹配参数：

```lua
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.03
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 25.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 90.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
```

有效性：

- `20/80` 已经可用；
- 微调到 `25/90` 后，用户现场确认“从角落二维码返回，误差很小”，因此 `25/90` 替换为新基线。

### 3.4 减少 ROS2 DDS 网络发现/SSH/热点对本机 ROS 图的影响

树莓派上的雷达、IMU、Cartographer、test01 都在本机运行，不需要 ROS 跨网络发现。

已在启动脚本和 `/etc/rc.local` 中加入：

```bash
export ROS_LOCALHOST_ONLY=1
```

作用：降低 ROS2 DDS 网络发现负载，减少联网/SSH 时对实时性和 CPU 的干扰。

### 3.5 修复 Cartographer 启动偶发 IMU fatal

问题现象：

```text
imu_tracker.cc: Check failed: (orientation_ * gravity_vector_).z() > 0. (0 vs. 0)
```

原因：Cartographer 启动太早时，WIT IMU 可能还没输出有效重力向量，导致初始化失败。

修复：在 launch 中用 `TimerAction(period=8.0)` 延迟启动：

```text
test01
cartographer_node
occupancy_grid_node
```

先启动：

```text
sllidar_node
wit_ros2_imu
robot_state_publisher
```

等 IMU / scan 稳定后再启动 Cartographer。

### 3.6 低负载日志与回修标记

不再让抓取脚本每条记录都 fsync，避免日志本身制造卡帧。

新增纯日志标记：

```text
pose_settle_warn
```

默认触发：最近 `10s` 内 pose 的 x/y 合成范围 `>=10cm` 或 yaw 范围 `>=3°`。用于后续分析“到点后静止回修”，不影响雷达输出，不控制小车。

## 4. 具体修改文件与函数

### 4.1 `scripts/rpi_sector_scan_filter.py`

角色：`/scan_raw -> /scan` 的扇区化 LaserScan 过滤节点。

有效修改：

1. 默认参数改为大转弯友好版：

```python
--front-half-deg default 150
--front-min-range default 0.80
--rear-min-range default 1.00
--rear-keep-every default 2
```

2. 新增/修改函数与逻辑：

```python
SectorScanFilter._ensure_geometry_cache(msg)
```

作用：缓存每个 beam 属于前方还是后方的判断，避免每帧重复角度计算，降低 Pi 负载。

```python
SectorScanFilter.on_scan(msg)
```

作用：按缓存后的扇区 mask 过滤近点/后方降采样，发布过滤后的 `/scan`。

```python
build_parser()
```

作用：提供环境变量可覆盖的默认过滤参数。

### 4.2 `scripts/rpi_light_pose_capture.py`

角色：低负载记录 `/pose`、`/tf`、`/scan_raw`、`/scan`、`/imu`。

有效修改：

```python
Cap.write(rec)
```

不再每条记录都 `fsync`，改为默认 `flush=1s`、`fsync=5s`，减少磁盘 IO 卡顿。

```python
Cap.on_pose(msg)
```

新增 `pose_hist` 窗口，并在 10s 内 x/y 或 yaw 回修超过阈值时写入：

```text
pose_settle_warn
```

### 4.3 `scripts/rpi_scan_timing_monitor.py`

角色：低负载 timing 摘要监测脚本。

关键函数：

```python
TopicStats.add_scan(...)
TopicStats.add_pose(...)
Monitor.flush_window()
Monitor.on_pose(...)
```

用途：记录 `/scan_raw`、`/scan`、`/imu`、`/pose` 的 1 秒窗口摘要，帮助判断 scan 卡帧、TF/pose 延迟、静止回修。

`Monitor.on_pose(...)` 新增 `pose_settle_warn` 标记。

### 4.4 `scripts/deploy_lidar_lowload_monitor.ps1`

角色：把低负载脚本和 sector filter 部署到树莓派。

关键动作：

```text
上传 rpi_sector_scan_filter.py
上传 rpi_light_pose_capture.py
上传 rpi_scan_timing_monitor.py
安装 sector_scan_filter executable
生成 start_scan_timing_monitor.sh
生成 start_light_capture_lowload.sh
```

### 4.5 `scripts/deploy_lidar_imu_start_delay.ps1`

角色：给 launch 增加 Cartographer 延迟启动保护。

核心修改：在 `test_grapher_3.launch.py` 中加入：

```python
from launch.actions import TimerAction

cartographer_start_delay = TimerAction(
    period=8.0,
    actions=[
        test01_node,
        cartographer_node,
        occupancy_grid_node,
    ],
)
```

### 4.6 树莓派 Cartographer 配置 `test02.lua`

路径：

```text
/home/ubuntu/cartographer/install/fishbot_grapher/share/fishbot_grapher/config/test02.lua
```

当前关键参数：

```lua
TRAJECTORY_BUILDER_2D.min_range = 0.8
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.03
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 100.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 25.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 90.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.78
```

### 4.7 树莓派启动文件 `/etc/rc.local`

有效修改：

```bash
export ROS_LOG_DIR=/home/ubuntu/cartographer/log
export ROS_LOCALHOST_ONLY=1
export FISHBOT_POSE_BIAS_X_CM=0
export FISHBOT_POSE_BIAS_Y_CM=0
export FISHBOT_POSE_BIAS_YAW_DEG=0
export FISHBOT_ORIGIN_LOCK_DELAY_S=45
ros2 launch fishbot_grapher test_grapher_3.launch.py &
```

## 5. 验证证据

### 5.1 用户现场反馈

- 大转弯一圈回来后定位准确；
- 从角落二维码返回，误差很小；
- `25/90` 微调版比 `20/80` 更好，因此替换为新基线；
- 当前误差可接受，可进入下一阶段。

### 5.2 现场日志检查

微调版日志：

```text
/home/ubuntu/codex_lidar_watch/logs/restart_bigturn_micro_20260515_040347.log
```

关键结论：

```text
scan rate: 10.01Hz
imu rate: 10.04Hz
pose jump blocked: 未出现
scan_filter 平均耗时: 约 8~9.7ms
Cartographer constraints: score 最低约 75%，平均约 80%+
```

后续又一次转一圈回来，当前 15 秒静止窗口：

```text
map->odom: 0.0cm 变化, yaw 0.0°
odom->base_link: 合成变化约 1.42cm, yaw 约 0.20°
```

`pose_settle_warn`：未触发。

## 6. 回退路径

### 6.1 当前正式基线

```text
logs/rpi_lidar_CURRENT_GOOD_READONLY/
logs/rpi_lidar_good_bigturn_micro_20260515_121238/
/home/ubuntu/codex_lidar_watch/snapshots/good_bigturn_micro_20260515_041241.tar.gz
```

### 6.2 上一个正样本，仅作为历史备份

```text
logs/rpi_lidar_good_bigturn_20260515_120002/
/home/ubuntu/codex_lidar_watch/snapshots/good_bigturn_20260515_040006.tar.gz
```

### 6.3 关键远程回退目录

```text
/home/ubuntu/codex_lidar_watch/rollback/before_bigturn_micro_20260515_040320/test02.lua
/home/ubuntu/codex_lidar_watch/rollback/before_lowload_monitor_20260515_042754
/home/ubuntu/codex_lidar_watch/rollback/before_rclocal_localhost_20260515_041908/rc.local
```

## 7. 后续边界

1. 不要继续大幅调雷达参数。
2. 雷达侧当前只允许：日志标记、只读抓取、轻量监测。
3. 若再次调 `min_range`、扇区过滤、Ceres 权重、motion_filter、pose_graph，必须先复制当前基线并记录回退路径。
4. 下一阶段主线应转向：
   - MSPM0 到点后稳定窗口；
   - 大转弯后稳定门控；
   - 轮式里程计 / 编码器融合；
   - 全局 map 坐标与车体系误差控制链。
5. 当前 `lidarCar/empty.c` 中若存在“到点稳定窗口”草稿，属于 MCU 控制侧，不属于本雷达侧基线；未烧录前不影响实车。

## 8. 学习卡

- 组件：Cartographer + sector scan filter + test01 pose bridge
- 角色：提供小车全局/局部位姿，输出给 MSPM0 用于导航控制
- 输入：`/scan_raw`、`/imu`、TF、Cartographer 配置
- 输出：`map->odom`、`odom->base_link`、`/pose`、UART 位姿帧
- 已解决问题：近点污染、大转弯后 x/y 漂移、Cartographer IMU 启动 fatal、日志/网络负载干扰
- 剩余风险：到点后 Cartographer 可能短暂回修；大转弯后 x 可能有小瞬态漂移
- 检查问题：如果小车已经物理到达起点，但雷达 10 秒后回修了 12cm，应该继续调雷达参数，还是让 MCU 的到点稳定窗口吸收这段回修？


## 9. 2026-05-15 14:37 连续两次二维码路线验证（min_score=0.78）

### 9.1 保存位置

```text
logs/return_qr_xdrift_root_20260515_143748/
logs/rpi_lidar_good_min_score_078_20260515_143748_READONLY/
logs/rpi_lidar_good_min_score_078_20260515_143748.tar.gz
logs/CURRENT_LIDAR_BASELINE.txt
```

包 SHA256：

```text
0df93bb6beb69556b14fc8216c08070931923124121328918abc6fbb27c50629
```

树莓派原始目录：

```text
/home/ubuntu/codex_lidar_watch/logs/return_qr_xdrift_root_20260515_143748
```

### 9.2 本轮现场结论

用户连续跑了两个二维码路线并返回起始点，现场反馈两次返回误差都很小；第二次把小车放到真实起始点中心后，日志最终误差仍在约 20cm 内。

本轮最终日志：

```text
last_pose = x +8cm, y +13cm, yaw -3.933deg
final_error = 15.26cm
```

停车后稳定窗口：

```text
last 10s: x range 1cm, y range 0cm, yaw range 0.449deg
last 30s: x range 4cm, y range 1cm, yaw range 0.785deg
```

### 9.3 约束证据

`min_score=0.78` 后，Cartographer 日志中没有接受低于 78 的约束。拉取到的节点日志整体统计：

```text
constraint count = 555
min score = 78.0
max score = 89.6
avg score = 83.23
max accepted translation = 0.22m
low score <78 = 0
```

这说明之前 `score≈77.1`、`translation≈4.06m` 这类弱约束大错配已经被挡掉。当前仍存在 0.18~0.22m 的边缘约束，但最终回起点误差已经明显下降。

### 9.4 当前调参判断

当前版本应作为下一阶段候选稳定版：

```lua
POSE_GRAPH.constraint_builder.min_score = 0.78
```

不要继续大幅改 `translation_weight`、`rotation_weight`、`motion_filter`、`optimize_every_n_nodes`。如果后续重复长路线仍超过 15~20cm，才考虑小步试 `min_score=0.79`，并必须保留当前 `0.78` 回退包。


## 10. 2026-05-15 连续三个二维码路线现场确认

用户现场确认：当前 `min_score=0.78` 版本连续跑三个二维码路线，返回起始点都准确。结合第 9 节完整抓取日志，当前雷达侧版本从“候选稳定版”升级为“正式 known-good 基线”。

代码只读快照：

```text
logs/rpi_lidar_20260515_three_qr_min_score_078_CODE_READONLY/
```

当前结论：

1. 保留 `POSE_GRAPH.constraint_builder.min_score = 0.78`。
2. 保留 `min_range=0.8`、扇区过滤、大转弯友好参数、低负载抓取脚本。
3. 暂停继续大幅调整雷达侧 Cartographer 参数。
4. 后续若再次出现长路线回起点误差超过 15~20cm，才允许小步 A/B 测试 `min_score=0.79`；测试前必须复制当前基线。
5. 下一阶段重点转向 MCU 控制侧：到点稳定窗口、路径点跟踪、编码器/IMU 短程协作。
