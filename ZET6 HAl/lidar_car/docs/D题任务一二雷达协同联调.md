# D 题任务一/任务二雷达协同运行补充

更新时间：2026-07-31

> 计分判据和现场入口以 [D题_V23_联调与计分矩阵.md](D题_V23_联调与计分矩阵.md)
> 为准。本页只补充小车运行细节。旧的 PG10 启动、PG9 维护、600 mm/s 首跑流程均已
> 废止；历史日志只作为证据，不作为操作步骤。
> 当前单向 Pi + MCU-local 校准仅可作兼容联调；正式计分前必须逐项关闭
> [D题_赛题合规差距清单_20260731.md](D题_赛题合规差距清单_20260731.md)。

## 按键

| 按键 | 作用 |
| --- | --- |
| PG13 | 任务一（抛投）启动；运行中再次按下为安全停车 |
| PG9 | 任务二（动态起降）启动；运行中再次按下为安全停车 |
| PG12 | 人工维护复位键：车辆静止满 12 s 后低有效长按 2 s；MCU 清除本地校准后直接广播三帧 `MAINTENANCE_RESET (0x85)`，不等待 Pi ACK |
| PG10 | 当前不参与启动、停车或任务控制 |

## 雷达坐标与地图方向

以下是 Pi 单向输入的原始地图/安装参考值，不保证等同于本次 `0x83` 后对外发布的
`FIELD_GLOBAL`。X/Y 单位为 cm，yaw 逆时针为正。坐标用于路线显示、B 点里程门和
A 点回程预备，不单独触发停车；本次 Delta、MCU 本地 `CalibrationId` 和校准后的
出站位姿才是现场坐标证据。

| 点位 | X (cm) | Y (cm) | 车体 yaw（约） | 用途 |
| --- | ---: | ---: | ---: | --- |
| A 起点/终点 | 0.0 | 0.0 | 0° | 起跑参考、回程灰度停车前置 |
| B | -151.3 | -8.1 | 0°（实测 -0.7°） | A-B 协同段结束/任务一雷达里程门 |
| C | -165.7 | 150.7 | 180°（实测 -178.1°） | 赛道上边回程方向参考 |
| D | -16.5 | 164.2 | 180°（实测 -179.4°） | D-A 回程段方向参考 |

B 点通常与 A 点航向同向，均接近 `0°`；C、D 点通常与 A/B 相反，均接近 `180°`
（`-180°` 与 `180°` 等价）。由坐标计算的 B-C、C-D 和 D-A 轮廓与
`docs/地图描述.png` 的约 `150 cm x 150 cm` 跑道一致，拐角约为直角。A 点回程若出现
数十厘米坐标残差，以灰度 A 标记为最终停车依据，不在 MCU 端擅自平移坐标。

Pi 到 MCU 的 UART4 为单向 `Pi TX -> MCU RX`，只发送原始位姿；当前不承担
`0x83/0x85` 控制或 ACK。MCU 向 LoRa `CAR_POSE(0x80)` 输出时，X 先减固定
`13 cm`，再镜像 X/Y、后叠加场地校准：
`X_lora=-(X_pi-13 cm)+DeltaX`、`Y_lora=-Y_pi+DeltaY`、`yaw_lora=yaw_pi`。
Pi 输入和本地雷达辅助始终保留原始 `X_pi/Y_pi`；MCU 只在出站帧写入本地
`CALIBRATED` 与 `CalibrationId`。校准前后差值相等不等于镜像正确，需抓 Pi 原始帧
和 LoRa 出站帧逐帧验证这三个公式。

启动键只要求 A 点全黑标记和新鲜 JY901；Pi 未上线、雷达跳变或 MCU 尚未校准时，小车
仍以本地低速进入循迹。连续三帧新鲜的原始 `FIELD_GLOBAL` 位姿与 MCU 本地有效
`CalibrationId` 同时满足后，才生成带该 ID 的 `0x81` 任务请求。V2.3 要求该 ID
与当前出站 `CAR_POSE` 一致，故未校准时不发送伪造的 `0x81`；任务请求失败或无线
ACK 超时只记录状态，不让循迹自动停车；尚未发完的 `0x81` 在安全停车时会取消。

## 速度和阶段门

主运动判据是车端传感器：JY901 提供实时 yaw/yaw-rate 和累计转弯方向，左右编码器
按轮增量累计路径距离；雷达坐标不参与横向控制，只作路线/里程交叉检查、A 点回程预备
和地面站协同显示。

- 任务一离开 A 使用约 `130 mm/s` 协同低速；任务二离开 A 使用约 `150 mm/s` 协同低速。
- 任务一在编码器累计路径达到约 `1550 mm` 后具备 B 进度；雷达连续两帧确认距 A
  至少 `155 cm` 只作坐标辅助。这两个门都在实测 B 点约 `151.5 cm` 之后，不能把
  A-B 伴飞段误判为已过 B。收到当前任务匹配的 `MISSION_STATUS(0x82)` 已经过
  `DROP_ACTION(6)` 且进入 `RETURN_HOME(11)`，才解锁约 `180 mm/s` 的回程速度包络；
  后续 `0x02` 只用于持续状态/丢包恢复。
- 任务二在 `INTERCEPT(3)`、`FOLLOW(4)`、`LAND_ALIGN(7)`、`DESCEND(8)` 和
  `ON_PLATFORM_5S(9)` 都保持 `150 mm/s` 协同低速；仅收到
  `PLATFORM_TAKEOFF(10)` 或之后的非 `ABORT(13)` 阶段才解锁约 `180 mm/s` 包络。
- 加速门在阶段成立时打开；飞行状态超过约 `1.5 s` 未更新或进入 `ABORT(13)` 时回到该任务的协同低速（任务一 `130 mm/s`、任务二 `150 mm/s`），但不会因无线状态单独停车。

## 雷达 A/B 辅助

若起跑门时已有 MCU 本地校准后的有效出站位姿，保存对应原始 A 样本为参考点；运行中只使用
同一 `CalibrationId` 会话内的新鲜原始位姿计算平面距离。起跑后才恢复的 Pi 位姿仍可触发
`0x81` 协同，但不会补抓已经离开的 A 点，避免将中途坐标错误当作 A 参考：

- 编码器累计路径距离达到约 `1550 mm` 时记录 `odometry_b_reached`，作为任务一和
  任务二均可用的 B 进度后备；连续两帧雷达距离 A 达到 `155 cm` 时记录 `radar_b_reached`，作为
  坐标辅助。单帧超过 `40 cm` 的位置跳变被丢弃；
- 回程进入 A 的 `45 cm` 半径时记录 `radar_a_prepare`，进入 `20 cm` 半径时记录
  `radar_a_stop_prepare`；
- 雷达预备不会单独停车。最终停车由灰度 A 标记、JY901 累计偏航和编码器累计路径距离
  共同判定；在已预备的
  A 半径内，雷达只允许短暂的原始全黑样本绕过稳定掩码延迟，避免车辆掠过 A 标记。

雷达位姿过期或校准 ID 改变时，辅助门失效，但循迹和既有 12 s 失线搜索逻辑继续运行。
推进释放后若连续 20 s 没有编码器里程增量，则记录 `progress_timeout` 并安全停车；正常
移动的一整圈仍以 90 s 看门狗为限，不能把 20 s 当作整圈时间上限。

## 无线接收

车端 UART5 按 V2.3 解析 CRC、版本、地址、长度和 ACK 标志：

- `MISSION_STATUS (0x82)`：`src=0x20,dst=0x10,len=12`，载荷为
  `TaskType, Stage, StatusFlags, MissionId, ErrorCode, Reserved, SourceTimeMs`。
  只有 `TaskType/MissionId` 与本次 `0x81 CAR_TASK_REQUEST` 匹配时才改变阶段。
- `FLIGHT_TELEMETRY (0x02)`：`src=0x20,dst=0x10,len=24`，使用载荷中的
  `ModeCode` 作为持续阶段；它没有任务 ID，因此只有当前 `0x81` 已 ACK 后才可作为
  调速依据，匹配的 `0x82` 是更强的会话确认。这样首帧 `0x82` 丢失时，2 Hz 广播仍可
  恢复阶段。
- `0x81` 请求在连续三个车端 `0-25 ms` 时隙各发送一次，同一 `Seq/Payload`，之后恢复
  `CAR_POSE(0x80)`。PG13/PG9 二次急停优先发送三帧 `MISSION_ABORT(0x84)`，等待标准 ACK
  但不影响已经关闭的电机。
- `CALIBRATION_SET(0x83)` 只在小车静止的维护窗口接收：MCU 校验后立即将
  `DeltaX/DeltaY/CalibrationId` 存入本地 RAM，并在下一可用车端时隙直接回
  `0x30 -> 0x40` 地面站 `ACK_ACCEPTED`。Pi 没有回传链路，绝不等待 Pi ACK；
  后续 LoRa `0x80` 才应用这组校准值，内部循迹/雷达原始坐标不变。

阶段编码：`0 IDLE`、`1 PRECHECK`、`2 TAKEOFF`、`3 INTERCEPT`、`4 FOLLOW`、
`5 DROP_ALIGN`、`6 DROP_ACTION`、`7 LAND_ALIGN`、`8 DESCEND`、
`9 ON_PLATFORM_5S`、`10 PLATFORM_TAKEOFF`、`11 RETURN_HOME`、
`12 HOME_LAND`、`13 ABORT`。

## 首轮联调顺序

1. 静止时按 `P` 保存日志；原始 Pi 位姿缺失或 MCU 未校准均不阻止本地起跑。需要协同时，
   先确认连续三帧原始位姿有效且 MCU 本地 `CalibrationId` 非零。
2. 不装载投掷物、桨叶保持安全，按 PG13；确认 `PG13_TASK1_REQUEST` 和
   `LOCAL_START_OK`。Pi 未接入时这一项仍必须能本地循迹。
   原始 Pi 位姿和 MCU 本地校准均就绪时，还应出现三次 `0x81`、对应 ACK 和
   `radar_a_captured`；任一未就绪时应记录 `task_coordination_deferred,RADAR_NOT_READY`，
   但小车必须继续循迹。
3. 观察 `radar_b_reached`；注入或接收匹配 `0x82 Stage=11`，确认只出现一次
   `coord_speed_unlocked`。
4. 回到 A，确认 `radar_a_prepare`/`radar_a_stop_prepare` 和
   `A_MARK_RETURN_RADAR_READY`，检查停车后不再发送任务请求。
5. 复位后用 PG9 重复，发送 `Stage=2`、`7`、`8`、`9` 时仍保持低速，发送
   `Stage=10` 后才提速；记录 `task2_b_within_15s` 或失败事件。
6. 运行中分别再次按 PG13、PG9，确认 `PG13_MANUAL_STOP`/`PG9_MANUAL_STOP`，并确认
   `task_tx_left=0`；PG10 不应产生任务或停车事件。

当前状态：联调固件已完成构建和静态协议检查，待烧录后按本文件的 PG13/PG9、雷达
点位和飞行阶段门进行连续小车/无人机实测；未完成实测的结论不得写成验收通过。
