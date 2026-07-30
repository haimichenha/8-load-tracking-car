# LoRa V2.2 UART5 双向链路验证（2026-07-30）

## 模式与安全边界

- 模式：`COMPETITION_LINE_FOLLOW`。
- 镜像：`LoRaProtocolDebug`，`UART5` 本地 115200 8N1。
- 诊断口：J-Link CDC `COM13`，115200 8N1。
- 电脑端远端无线：CH340 `COM12`，115200 8N1。
- 本测试只收发 V2.2 固定夹具；不执行任务、ACK、校准转发或电机控制。主循环持续
  `MotorSafe_ForceOff()`。

## 已确认接线

| STM32F103ZET6 | 无线模块 | 方向 |
| --- | --- | --- |
| `PC12` / UART5 TX | RX | MCU -> 模块 |
| `PD2` / UART5 RX | TX | 模块 -> MCU |
| GND | GND | 共地 |

## 通过证据

命令：

```powershell
.\scripts\test_lora_protocol_link.ps1 -DiagPort COM13 -WirelessPort COM12 -BaudRate 115200
```

原始记录：`logs/lora_protocol_link_20260730-120709.log`。

1. 诊断命令 `P` 令 MCU 发送规范 `CAR_POSE` 测试向量；COM12 收到 33 B，逐字节相同：
   `AA 55 02 80 30 10 42 00 16 ... B2 2D`，`TX_PASS=1`。
2. COM12 反向发送规范 `CAL` 测试向量；MCU 日志为
   `type=131,src=64,dst=48,seq=22,flags=1,len=12,accepted=1`，`RX_PASS=1`。
3. 汇总：`RESULT,PASS,tx_exact_vector=1,rx_crc_valid=1,motors_safe=1`。

## 范围外事项

此结果只将 LoRa 透明传输绑定到 UART5。竞赛任务仍需实现并单独验证：Pi 位姿入口、真实
`CAR_POSE` 10 Hz 与 0--25 ms 发送槽、物理按键任务请求、ACK 重传、校准状态机及运行态安全策略。
