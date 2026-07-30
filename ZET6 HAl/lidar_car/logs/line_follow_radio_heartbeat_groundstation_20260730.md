# 2026-07-30 地面站 HEARTBEAT 空口验收

- **模式**：`COMPETITION_LINE_FOLLOW` 的维护无线测试镜像；车辆未由本项测试驱动电机。
- **小车镜像**：`LineFollowJY901Debug`，构建后 `FLASH=26252 B`，J-Link 分段比较均为 `matched`。
- **小车发送**：UART5（PC12 MCU TX -> 无线模块 RX，115200 8N1），从上电起每 500 ms 发送
  V2.2 `HEARTBEAT (0x03)`，`src=0x30`、`dst=0x10`、无 ACK。
- **固定 Payload（8 B）**：`DeviceStatus(u8)`、`ErrorCode(u8)`、`Reserved(u16=0)`、
  `UptimeMs(u32, big-endian)`；整帧为 19 B，CRC16/CCITT-FALSE 覆盖 Version 至 Payload。

## 接收端证据

地面站为 `raspi@192.168.0.133`，接收进程
`/home/raspi/.local/bin/ground_station_ui.py` 持有 `/dev/ttyUSB0`，本地串口配置为 115200。
其 `decode_heartbeat()` 要求长度为 8 B、`Reserved=0`，与本次发送格式一致。

修复前旧空载帧在 `20:59:34` 被明确丢弃：

```text
HEARTBEAT payload must be 8 bytes, got 0
```

修复镜像运行后的地面站统计如下；`valid` 持续增加，而 `crc`、`length` 和 `payload` 丢弃计数保持不变：

```text
21:03:10  valid=2101 crc=0 length=0 payload=1689 dropped=332
21:03:52  valid=2185 crc=0 length=0 payload=1689 dropped=332
```

42 秒内增加 84 个合法帧，即约 2 Hz；没有新的 `HEARTBEAT payload must be 8 bytes` 告警。

## 结论与边界

`HEARTBEAT` 的透明无线收发、帧长、保留字段和 CRC 已通过地面站实测。`car_age=-1` 仍为预期：
当前维护帧不是 22 B 的 `CAR_POSE (0x80)`，不携带平台中心位置、校准 ID 或位姿有效标志。
不得将本记录作为 10 Hz 位姿、LoRa 时隙、任务请求或陆空协同任务验收。
