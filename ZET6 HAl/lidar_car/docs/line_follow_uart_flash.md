# 循迹固件烧录经验与串口恢复方案

当前任务一/二的运行参数和联调顺序以
[D题任务一二雷达协同联调.md](D题任务一二雷达协同联调.md) 为准；本文件只说明烧录恢复。
烧录成功只证明镜像写入和校验完成；正式赛题合规仍以
[D题_赛题合规差距清单_20260731.md](D题_赛题合规差距清单_20260731.md) 为准。

## 默认烧录方式

默认使用 J-Link SWD `50 kHz`，不是 UART ROM Bootloader。2026-08-01 已用该方式
烧录当前 `LineFollowMissionDebug`：J-Link 连上 `STM32F103ZE`、`VTref=3.30 V`，六个
Flash 段全部 `matched`，随后执行复位和 `go`；COM13 `P` 状态确认应用已启动。

`LineFollowMissionDebug` 在本次工作中始终可以正常构建。下列事项仍是诊断结论：

- J-Link 被 USB 识别、且显示 `VTref=3.30 V`，并不代表 SWD 可用。
- `COM13` 是 J-Link CDC UART。它能交换应用诊断信息，但 J-Link 仍可能
  无法初始化目标 DAP。
- 只有目标连接成功且下载/段校验通过，才算烧录成功。VS Code 报
  `localhost:2331` 超时，表示 GDB Server 已在连接目标前退出，不是应用
  固件本身的错误。

工程没有关闭 SWD。`GPIO_Remap_SWJ_JTAGDisable` 仅释放 JTAG 专用引脚，
PA13/PA14 的 SWD 功能仍保留。

## J-Link SWD 烧录（默认）

本次已在 `STM32F103ZE`、`VTref=3.30 V`、SWD `50 kHz` 下完成下载和逐段
`compare-sections` 校验。车辆断电或架空车轮后，在项目根目录的 `cmd.exe` 中执行：

```bat
cd /d "F:\keil5\stm\ZET6 HAl\lidar_car"
powershell -NoProfile -ExecutionPolicy Bypass -File "scripts\jlink_flash.ps1" -Configuration LineFollowMissionDebug -SwdSpeedKhz 50
```

该命令会重新配置和构建 `LineFollowMissionDebug`，写入 `lidar_car.elf`，逐段比对
Flash，并在成功后复位、运行目标。脚本会捕获 GDB 正常的 `Resetting target` 输出，避免
将其误判为烧录失败。输出中的每个段都应显示 `matched`；只有脚本退出码为 `0` 才可开始
实车测试。

## UART ROM Bootloader（仅备用）

诊断串口直接使用 STM32F103 的 USART1：

- PA9：USART1 TX
- PA10：USART1 RX
- 主机端：通常为 `COM13`（`JLink CDC UART Port`）

这正好也是 STM32F103 ROM Bootloader 的 USART1 引脚。因此当 SWD 无法
连接时，可通过串口烧录恢复。

1. 关闭 VS Code 调试和所有占用目标 COM 口的程序。
2. 断开电机电源或架空车轮，避免复位后误动作。
3. 在任意 `cmd.exe` 窗口先切换到项目根目录，再执行：

   ```bat
   cd /d "F:\keil5\stm\ZET6 HAl\lidar_car"
   scripts\flash_line_follow_uart.cmd COM13
   ```

   省略 `COM13` 时默认使用该端口；若 Windows 分配了其他端口，传入实际
   的 `COMx` 即可。
4. 脚本构建完成后会暂停。此时设置 `BOOT0=1`、保持 `BOOT1=0`，然后复位
   或重新上电目标板，按任意键开始下载。
5. 等待 STM32CubeProgrammer 报告下载和校验成功。
6. 设置 `BOOT0=0`，再复位或重新上电，启动新应用。

脚本会配置和构建 `LineFollowMissionDebug`，然后调用本机
STM32CubeProgrammer 2.23.0 CLI 写入并校验 HEX 镜像。脚本不会在写入后
自动复位，因为 `BOOT0=1` 时复位只会再次进入 ROM Bootloader。

## 当前固件烧录后验证项

- PG13 启动任务一，PG9 启动任务二；运行中再次按 PG13 或 PG9 均急停并发送
  `MISSION_ABORT(0x84)`。PG12 在车辆停止后低有效短按，经 50 ms 消抖后 MCU 直接清除
  本地人工校准并在三个连续车端时隙广播 `MAINTENANCE_RESET(0x85)`；三帧完成后可重复
  短按，不等待 Pi ACK。
  PG10 不参与任何任务或急停。
- `LineFollowMissionDebug` 的 PG12 固定为短按，不要求静止 12 s 或长按。烧录后用串口
  `P` 确认 `pg12_short_press_only=1`；`maint_reset_attempts`、`maint_reset_successes`、
  `maint_reset_reject_running`、`maint_reset_reject_busy` 与 `maint_last_result` 可回查刚才
  的按键结果。V2.3 附录的人工作业时序是外部赛题条款，当前固件不提供开关切回该行为，
  也不能替代 Pi 清除/ACK 的正式闭环。
- 任务一离开 A 约 `130 mm/s`，前轮 LADRC 正向 PWM 上限 `45%`；任务二约 `150 mm/s`，
  因 B 点实测约 13 s 而将正向上限提高到 `55%`。这不是固定加 10 PWM，速度达到目标时
  LADRC 仍会自动降占空比；雷达 B/C/D 坐标、B 后速度门和飞行阶段门见当前联调文档。
- Pi -> MCU UART4 是单向原始位姿输入。对外发送的 `CAR_POSE` 仅在 MCU LoRa 出站阶段
  计算 `X_lora=-(X_pi-13 cm)+DeltaX`、`Y_lora=-Y_pi+DeltaY`，保持 `yaw_lora=yaw_pi`，并写本地
  `CALIBRATED/CalibrationId`；原始 Pi/radar 坐标和本地循迹不应用镜像或 Delta。必须
  同时抓 Pi 原始帧与 LoRa 出站帧验证公式，Pi 不得重复镜像。
- 静止时发送 `0x83` 后，应先收到 MCU 直接回给地面站的 `ACK_ACCEPTED`，再观察至少
  三帧同一 CalibrationId 的校准后 `CAR_POSE`；相对校准前出站 X/Y 的增量必须等于
  `DeltaX/DeltaY`。这里不得等待或伪造 Pi ACK。
- PG13/PG9 起步后的空侧联调顺序必须是三帧成功出站、当前 `CalibrationId` 的
  `CALIBRATED 0x80`（本地已经起步时同时带 `CAR_RUNNING`），再是同 `Seq/MissionId/CalibrationId` 的三帧
  `0x81`，最后是空侧 `0x11 ACK`。状态中应看到
  `task_pose_preamble_tx=3`、`task_pose_preamble_required=3` 和
  `radio_uart_err=0`；任一 Pi 位姿失鲜或前导中插入地面 ACK 都会重新计数。
- 中心传感器采用精确 X4+X5 判定；误差到差速的映射连续化，并在任务层限制
  每 20 ms 的基速/差速目标变化，避免进入或退出弯道时跳变。
- 失线后保留最近有效转向方向，最多搜线 12 秒；只有中心传感器捕获并连续 10
  个 20 ms 控制周期（约 200 ms）的居中 `TRACK`/`WIDE` 才恢复正常循迹。
- 未确认回到 A 点时，短暂陀螺/编码器异常和起跑离开 A 超时只记录告警、继续
  循迹；自动停车仅来自确认 A 点、运行满 90 秒或失线搜线超时。当前
  `LineFollowMissionDebug` 禁用 UART `S`，物理急停仍由运行中第二次按 PG13/PG9 完成。
- 若推进释放后连续 `20 s` 没有编码器里程增量，固件记录
  `progress_timeout,NO_ENCODER_PROGRESS_20S` 并安全停车。这是卡滞保护，不缩短正常
  循迹的 90 s 整圈上限。
- 运行中不再同步输出完整 `LF,event=sample` 记录，避免 115200 串口忙等阻塞
  控制周期。RAM 冻结记录仍以 100 ms 保存，停车后发送 `F` 导出分析。
- 紧弯允许内侧轮有限反向目标，当前内侧目标下限为 `-800 cps`；急弯优先调差速
  持续时间，不全局放大 LADRC。

每次实车后导出冻结日志，检查失线保持时长、保持差速、重获计数和反向轮
目标记录，再决定是否继续提高转弯力度。
