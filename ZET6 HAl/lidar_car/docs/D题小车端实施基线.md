# D 题小车端实施基线

> 2026-07-31 起，任务一/二的现行按键、低速、飞行阶段、计时和得分判据以
> [D题_V23_联调与计分矩阵.md](D题_V23_联调与计分矩阵.md) 为准。本文只保留
> 当前实现、接口边界和可追溯证据；不单独声明赛题通过。
> 正式计分前还必须关闭 [D题_赛题合规差距清单_20260731.md](D题_赛题合规差距清单_20260731.md)
> 的 G1--G7。尤其是单向 Pi 的 MCU-local 校准属于当前兼容路径，不能替代已发布
> V2.3 中 Pi 的平台中心与 ACK 责任。
> PG12 的现行联调默认与附录验收兼容模式是互斥的编译选择：默认短按，不默认附加
> 静止 12 s 或长按门槛；后者只在显式验收镜像中启用。

## 1. 目标与范围

本文件定义 D 题陆空协同系统的小车端运行态。八路灰度、直接接入的 JY901 yaw/yaw-rate
和前轮编码器是车辆本地运动闭环；雷达/树莓派只单向提供原始位姿给 MCU，由 MCU 负责
对外发布的平台中心坐标和校准状态。
小车不负责无人机的起飞、投放、着陆判定或飞控状态机。

通信唯一依据为 `F:\keil5\stm\ZET6 HAl\lidar_car\docs\D题_通用通信与接口规范_v2.3.docx`；
在线帧 Version 仍为 `0x02`。
角色实现约束见 `F:\keil5\stm\docs\D题_Agent修改提示词_v2.2.md`。题目澄清见
[D题 回复.txt](D题%20回复.txt)。

## 1.1 当前雷达路线参考

下表是 Pi 本地原始地图/安装参考；现场 `0x83` 校准后，MCU 才将它映射为对外
`FIELD_GLOBAL`。X/Y 单位为 cm，yaw 逆时针为正。MCU 以起跑时捕获的原始 A 样本
做相对距离辅助，不依赖固定 A=`(0,0)` 启动车辆或停车。

| 点位 | X (cm) | Y (cm) | 航向约定 | 当前用途 |
| --- | ---: | ---: | --- | --- |
| A | 0.0 | 0.0 | 0° | 起跑参考、回程灰度停车前置 |
| B | -151.3 | -8.1 | 0°（实测 -0.7°） | 任务一 B 里程门 |
| C | -165.7 | 150.7 | 180°（实测 -178.1°） | 上边路线参考 |
| D | -16.5 | 164.2 | 180°（实测 -179.4°） | 回程路线参考 |

B 与 A 的车体方向接近 `0°`，C/D 与其相反、接近 `180°`；负角度是同一方向的
`[-180°,180°)` 表示。B-C、C-D、D-A 的实测轮廓与 `docs/地图描述.png` 的约
`150 cm x 150 cm` 跑道相符。雷达只做里程/停车预备和地面站显示；JY901 累计偏航与
编码器累计路径距离才是车辆运动和 A 点判定的主辅助量，最终仍需灰度标记。

## 2. 当前硬件事实

| 子系统 | 已确认事实 | 状态 |
| --- | --- | --- |
| 前轮 TB6612 | PWM `PA2/PA3`，方向 `PE2-PE5`，STBY `PE6` | 当前运动基线 |
| 前轮编码器 | 左 `PA0/PA1 TIM5`，右 `PA6/PA7 TIM3` | 当前反馈基线 |
| 八路灰度循迹 | 地址线 `AD0/AD1/AD2=PC0/PC1/PC2`，数字 `OUT=PG0` | Yahboom 74HC4051 地址扫描接口；X1--X8 已实机验证为 `0x01`--`0x80`，白底基准 `0x00`；X4 左偏 / X4+X5 居中 / X5 右偏的误差符号已验证 |
| 后轮 TB6612 测试 | PWM `PE13/PE14`，方向 `PF1-PF4`，STBY `PB9` | 仅架空四轮测试；与灰度主组无已知引脚冲突 |
| 后轮编码器 | 左 `PB6/PB7`，右 `PC6/PC7` | 目前仅输入轮询，TIM4/TIM8 闭环待完成 |

后轮 STBY 已改接 PB9，灰度主组 `PC0/PC1/PC2 + PG0` 不再与后轮驱动冲突。`FourWheelTb6612Debug` 仍只用于电机带起，因为它不初始化灰度解码、循迹控制和任务状态机；比赛固件需在四轮回归通过后另行验证。

### 位姿与校准物理拓扑

`UART4 PC11 (MCU RX) <- Pi TX` 是当前 Pi 链路的唯一已用方向：Pi 以 10 Hz 发送原始
`CAR_POSE` 或兼容雷达帧，MCU 不向 Pi 发送校准、复位或 ACK。实际校准闭环如下：

`地面站 0x83 -> MCU 本地 RAM(DeltaX/DeltaY/CalibrationId) -> MCU 直接地面 ACK`

`Pi 原始位姿 -> MCU 内部原始状态 -> MCU LoRa 出站校准 CAR_POSE`

因此 Pi ACK 不得作为 `0x83`、PG12 或 `0x85` 的前置条件；原始 Pi/radar 坐标只用于
内部雷达辅助和相对距离，不能被 MCU 本地 Delta 回写或替换。

## 3. 灰度循迹与速度策略

1. 八路灰度映射已验证：白底 `0x00`、居中 X4+X5=`0x18`、左偏 X4=`0x08`、右偏 X5=`0x10`；权重 `-7/-5/-3/-1/+1/+3/+5/+7` 的负值表示黑线在车体左侧。`LineFollowMissionDebug` 由灰度误差、JY901 yaw/yaw-rate 和前轮双 `SpeedLadrc_t` 形成有界差速，后轮同侧开环跟随。后轮编码器未标定，不能称为四轮速度闭环。
2. A 点全黑 `active_mask=0xFF` 只是位置标记。PG13 启动任务一，PG9 启动任务二；运行中再次按对应任务键均进入安全停车。PG10 不初始化、不读取，也不产生启动/停车事件。PG12 用于停车后的短按人工维护复位：50 ms 消抖后 MCU 直接清除本地 Delta、校准标记和 CalibrationId，再广播 `0x85`，不等待 Pi ACK。权威附录的静止/长按时序是外部赛题条款，不对应当前固件分支。
3. 任务一离开 A 以约 `130 mm/s` 协同低速循迹，任务二以约 `150 mm/s` 协同低速循迹。两项任务均可用编码器累计约 `1550 mm` 或雷达连续两帧确认距 A 至少 `155 cm` 作为 B 进度。任务一需见 `DROP_ACTION(6)` 且进入 `RETURN_HOME(11)` 后才允许约 `180 mm/s` 回程速度；任务二在 `INTERCEPT(3)`、`LAND_ALIGN(7)`、`DESCEND(8)`、`ON_PLATFORM_5S(9)` 均不得提速，只能在 `PLATFORM_TAKEOFF(10)` 或之后的非 `ABORT(13)` 阶段提速。
4. 灰度仍是主路径和最终 A 停车依据；JY901 累计偏航加编码器累计路径距离用于防止
   首次经过 A 时误停。雷达使用当前 A/B/C/D 坐标做里程交叉检查、A 点回程预备和地面站
   显示，不单独停车。
5. 失线进入 `LOST_HOLD` 后按最近四帧方向寻线，最多持续 12 s；只有中心传感器捕获并连续 10 个 20 ms 控制周期（约 200 ms）稳定重获后才恢复循迹，超时才安全停车。陀螺仪/编码器短时异常只记录告警；赛题镜像保留 90 s 整圈看门狗，并增加 20 s 无编码器里程增量保护，分别记录 `lap_complete/lap_timeout` 与 `progress_timeout`。
6. 当前转弯差速允许内侧轮有限反向目标（下限 `-800 cps`），并通过每 20 ms 的目标斜率限制平滑变化；急弯优化以差速持续时间为主，不全局放大 LADRC。
7. 无线阶段由 `MISSION_STATUS(0x82)` 绑定任务，再用 `FLIGHT_TELEMETRY(0x02)` 持续补充；`ABORT(13)` 或阶段超过约 1.5 s 未更新时回到协同低速，不因无线状态单独停车。

PG13/PG9 的本地启动只检查 A 点灰度标记和 JY901 新鲜度，不检查雷达、Pi 或
`CalibrationId`。Pi 位姿不可用时记录 `task_coordination_deferred,RADAR_NOT_READY` 并继续
循迹；连续三帧合格的原始位姿与 MCU 本地有效 `CalibrationId` 同时出现后，才补发
不影响推进的 `0x81` 协同请求。

## 4. LoRa 与协同边界

树莓派以 115200 8N1、10 Hz 向 MCU 单向发送原始 `FIELD_GLOBAL` 雷达/SLAM 位姿。
MCU 校验后成为唯一 LoRa 发射者和校准状态所有者：

- 广播 `CAR_POSE`（0x80）10 Hz，坐标单位 cm、yaw 单位 0.1 度。Pi 输入保持原始
  `X_pi/Y_pi/yaw/velocity/SourceTimeMs`；MCU 只在 LoRa 出站计算
  `X_lora = -(X_pi - 13 cm) + DeltaX`、`Y_lora = -Y_pi + DeltaY`，并保持 `yaw_lora=yaw_pi`。
  Pi 不得重复镜像或 Delta；
- MCU 用本地状态覆盖出站 `CALIBRATED/CalibrationId`，并按真实循迹状态写入
  `CAR_RUNNING`。入站 Pi 的校准标记和 ID 不参与内部坐标、协同就绪或对外帧；
- 每个 100 ms 周期仅在 0-25 ms 的车端槽发射；30-55 ms 留给机端回应，55-100 ms 静默；
- PG13/PG9 先发起本地任务；仅当连续三帧原始位姿有效，且 MCU 本地校准状态为同一有效
  `CalibrationId` 后，才以固定任务 ID/序列发送三次 `CAR_TASK_REQUEST(0x81)`；
  Pi/雷达缺席只延后此空地会话，不得阻止电机推进；
- 运行中 PG13/PG9 二次按键立即关闭电机，并优先在连续三个车端时隙发送
  `MISSION_ABORT(0x84)`（`URGENT|ACK_REQUIRED`）；等待 ACK 只用于遥测，不延迟停车。
- 地面 `0x83` 只在车辆静止时接受。MCU 校验后立即更新本地
  `DeltaX/DeltaY/CalibrationId`，并在下一个车端时隙向地面回 `ACK_ACCEPTED`；
  不向 Pi 转发，也不等待 Pi ACK。任务中不得更改 `CalibrationId`；
- 当前物理维护事件为 PG12 低有效短按（50 ms 消抖）且 `CAR_RUNNING=0`；MCU 立即清除
  本地 Delta、`CALIBRATED` 与 CalibrationId，并在连续三个车端时隙广播同一
  ResetId/Seq/Payload 的 LoRa `0x85`。该事件不访问 Pi，后两帧仅携带
  `RETRANSMISSION`；三帧完成后可再次短按并分配新的 ResetId。
- V2.3 附录的静止/长按要求不是当前固件的可选分支；它作为外部赛题条款和 Pi
  清除/ACK 合规缺口一并记录，不能将短按事件标记为该条款的验收结果。
- 任务中小车持续循线，不因 LoRa ACK、遥测延迟或地面站显示而重复启动或停车；仅灰度
  A 标记、20 s 无编码器进展保护、90 s 看门狗、12 s 失线超时和 PG13/PG9 二次按键能结束车辆运动。

LoRa 透明无线模块已于 2026-07-30 完成独立双向台架验证：MCU 采用 `UART5`、115200 8N1，
`PC12=MCU TX -> 无线模块 RX`，`PD2=MCU RX <- 无线模块 TX`。PC 侧远端模块为 COM12，
接收到了逐字节一致的 33 B `CAR_POSE` 官方测试向量；反向发送 23 B `CAL` 官方测试向量后，
MCU 记录了 CRC 有效且字段匹配的 `rx_ok`。当前镜像已启用 10 Hz 位姿转发、车端时隙、
`0x81` 任务重传、`0x82/0x02` 阶段接收和 `0x84` 急停通知；正式现场验收仍需实测。
UART5 不得再同时接 Pi/Nano；Pi 位姿入口固定为 UART4。

当前 `LineFollowMissionDebug` 在没有新鲜 Pi 位姿且小车静止时每 500 ms 发送 `HEARTBEAT(0x03)`；它采用
固定 8 B 载荷 `DeviceStatus(u8) + ErrorCode(u8) + Reserved(u16=0) + UptimeMs(u32, big-endian)`。
一旦 UART4 位姿新鲜，心跳让位给 10 Hz `CAR_POSE(0x80)`；心跳不能替代其 22 B 载荷、车端时隙和校准门禁。

2026-07-30 已由地面站 `raspi@192.168.0.133` 的 `/dev/ttyUSB0` 实测接收：8 B 修复镜像运行后，
`valid` 计数按约 2 Hz 增长，而 CRC、长度和 payload 丢弃计数不再增长。此项仅为维护心跳
台架验收，`car_age=-1` 在未发送 `CAR_POSE` 时属预期结果；证据见
`logs/line_follow_radio_heartbeat_groundstation_20260730.md`。

V2.3 位姿转发代码现已接入同一镜像：UART4 首选 Pi 的原始
`CAR_POSE(src=0x31,dst=0x32,len=22)`，校验 CRC、位置/yaw 有效位、yaw 范围和速度无效
哨兵值后，将其作为 MCU 内部原始状态。Pi 的 `CALIBRATED/CalibrationId` 不被作为校准
权威；MCU 经 UART5 以 `src=0x30,dst=0x10` 每 100 ms 广播时，才按本地状态写入
`X_lora/Y_lora/CALIBRATED/CalibrationId`。为兼容当前物理 Pi 的 14 B `FA...AB` 本地
桥接帧，MCU 也可将其封装为原始预检 `CAR_POSE`；该兼容路径从首个有效旧帧启动 MCU
转发 epoch 作为临时 `SourceTimeMs`。MCU 依据真实循迹状态改写 `CAR_RUNNING` 位；
Pi 数据超过 250 ms 后，车辆跑动中保持 LoRa 静默，静止时退回维护心跳。

2026-07-30 的后续回归中，UART4 RXNE 环形缓冲消除了前台轮询造成的帧失步；MCU 已从 14 B 兼容流向地面站
持续产生新鲜 `CAR_POSE`（`car_age<=0.098s`，约 10 Hz，UART4 原始输入约 20 Hz）。静止原始帧解码为
`X=6 cm,Y=8 cm,yaw=9.6°`，说明 Pi 雷达零位仅有数厘米残差而非 MCU 载荷字节序错误；详见
`logs/car_pose_legacy_bridge_groundstation_20260730.md`。该记录只证明原始位姿显示链路，
不解除 CRC/时隙/任务验收要求。当前实装 Pi 仍仅输出 `FA...AB` 旧桥接帧；在当前单向
硬件构型下，PG12 和 `0x83` 均由 MCU 本地处理，Pi 不保存、也不确认 CalibrationId。
因此当前默认中 PG12 合格短按必须清除 MCU 本地校准并直接三帧广播 `0x85`；若切换
V2.3 附录的静止 12 s + 长按 2--3 s 是外部赛题条款，不取代当前短按触发。

## 5. MaixCam V2.3 协同边界

MaixCam Pro 只与飞控进行 `CAMERA_MODE`、`CAMERA_TARGET`、`CAMERA_ACTION` 和
`CAMERA_ACTION_RESULT` 会话。飞控在接受小车任务后创建 `ModeSeq`，并以
`TaskType + MissionId + ModeSeq` 约束视觉目标和投放动作。小车 MCU 与树莓派：

- 先产生并保持本地 `TaskType/MissionId`；只有 MCU 已取得本地有效
  `CalibrationId` 后才发送 `CAR_TASK_REQUEST`，并在可用时转发经 MCU 校准的 `CAR_POSE`；
- 不生成或解析 `ModeSeq`，不解析相机 TARGET/RESULT，不直接驱动相机或投放舵机；
- 不因相机模式 ACK、视觉目标丢失、投放 RESULT 或遥测延迟停止、重启或改变循迹。

雷达/Pi 位姿可作为无人机外部跟踪输入；这不改变小车以灰度、JY901 和编码器为主的
运动控制权责。MaixCam 仅在飞控已建立有效会话后的末段提供机体系纠偏。

## 6. 运行态与验收

```text
IDLE -> 车上按键选任务 -> 本地灰度/JY901/编码器循迹 -> 完成或安全停止
                         \-> Pi 原始位姿 + MCU 本地校准可用时建立 0x81/0x82/0x02 空地会话
```

最小验收证据：

| 项目 | 必须记录 |
| --- | --- |
| 灰度阵列 | X1--X8 地址扫描位号、`all_white_mask`、地图白黑极性、误差符号、丢线策略；记录每头 `CHANNEL_SWITCH` 日志 |
| A-B | 场地版本、至少多次用时、选定速度和 15 s 余量 |
| 电机 | 指令方向、实际方向、编码器符号、最大安全 PWM |
| LoRa | 帧类型、序列、CRC、位姿年龄、时隙时刻、重发次数 |
| 任务按钮 | 任务类型、任务 ID、单次触发和去重结果 |
| 空地会话边界 | 小车任务 ID 与飞控/相机状态匹配记录；无小车直接相机控制 |

地面站只显示场地、小车、无人机、链路新鲜度和校准/任务状态。任务运行期间不应提供小车启动、停止、调参或代码下发控制。
