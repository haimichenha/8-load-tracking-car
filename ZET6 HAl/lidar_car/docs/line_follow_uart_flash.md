# 循迹固件烧录经验与串口恢复方案

## 本次结论

`LineFollowMissionDebug` 在本次工作中始终可以正常构建，问题发生在
SWD 传输链路，而不是源码或镜像：

- J-Link 被 USB 识别、且显示 `VTref=3.30 V`，并不代表 SWD 可用。
- `COM13` 是 J-Link CDC UART。它能交换应用诊断信息，但 J-Link 仍可能
  无法初始化目标 DAP。
- 只有目标连接成功且下载/段校验通过，才算烧录成功。VS Code 报
  `localhost:2331` 超时，表示 GDB Server 已在连接目标前退出，不是应用
  固件本身的错误。

工程没有关闭 SWD。`GPIO_Remap_SWJ_JTAGDisable` 仅释放 JTAG 专用引脚，
PA13/PA14 的 SWD 功能仍保留。

## 串口备用烧录

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

## 本轮控制验证项

- PG10 保持电机启动/停止；PG9 保持人工校准。
- 中心传感器采用精确 X4+X5 判定；误差到差速的映射连续化，并在任务层限制
  每 20 ms 的基速/差速目标变化，避免进入或退出弯道时跳变。
- 失线后保留最近有效转向方向，最多搜线 12 秒；连续 3 帧居中的 `TRACK`
  或 `WIDE` 才恢复正常循迹。
- 未确认回到 A 点时，短暂陀螺/编码器异常和起跑离开 A 超时只记录告警、继续
  循迹；自动停车仅来自确认 A 点、运行满 3 分钟或失线搜线超时。PG10 与 UART
  `S` 仍可人工停车。
- 运行中不再同步输出完整 `LF,event=sample` 记录，避免 115200 串口忙等阻塞
  控制周期。RAM 冻结记录仍以 100 ms 保存，停车后发送 `F` 导出分析。
- 紧弯允许内侧轮有限反转。`5200 cps` 差速上限和 `-800 cps` 内侧目标仍是
  分阶段实车验证参数，不应视为历史验证过的基线。

每次实车后导出冻结日志，检查失线保持时长、保持差速、重获计数和反向轮
目标记录，再决定是否继续提高转弯力度。
