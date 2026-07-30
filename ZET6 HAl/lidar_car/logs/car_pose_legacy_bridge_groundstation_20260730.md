# 2026-07-30 UART4 雷达位姿到地面站联调

## 模式与边界

- MCU 镜像：`LineFollowJY901Debug`，`FLASH=29948 B`；J-Link 下载后的每一段比较均为 `matched`。
- 当前 Pi 物理输出为历史本地 14 B 帧：`FA | sign+x[3] | sign+y[3] | sign+yaw10[3] | AB`。
- MCU 将该帧封装为 V2.2 `CAR_POSE(0x80)` 并由 UART5 LoRa 广播；它固定为
  `POSITION_VALID|YAW_VALID`、`CALIBRATED=0`、`CalibrationId=0`、`Vx=Vy=0x7FFF`。
- 该迁移输入不含 Pi CRC、校准 ID、速度和 Pi SourceTimeMs，故本记录只允许预检显示，禁止用于
  飞机任务接受或正式校准验收。

## MCU J-Link CDC（COM13）证据

```text
t=10666 ms: pi_pose_raw_bytes=2401, pi_pose_legacy_ok=62,
             pi_pose_v22_ok=0, pi_pose_fresh=1, pi_pose_age_ms=220,
             radio_pose_tx_count=105, radio_pose_drop_count=0
t=11945 ms: pi_pose_raw_bytes=2630, pi_pose_legacy_ok=68,
             pi_pose_v22_ok=0, pi_pose_fresh=0, pi_pose_age_ms=298,
             radio_pose_tx_count=117, radio_pose_drop_count=0
```

结论：UART4 确有雷达 Pi 原始字节和有效的 `FA...AB` 帧；当前并未收到 Pi 原生 V2.2 输入。
由于旧桥接流出现大于 250 ms 的间隙，`pi_pose_fresh` 会短暂变化；MCU 不会在车辆跑动且位姿陈旧时
发送维护心跳。

## 地面站（`raspi@192.168.0.133`，`/dev/ttyUSB0`）证据

```text
21:19:00 valid=3969 crc=0 length=0 payload=1689 ... car_age=0.000s
21:19:10 valid=4079 crc=0 length=0 payload=1689 ... car_age=0.200s
21:19:20 valid=4185 crc=0 length=0 payload=1689 ... car_age=0.199s
```

`valid` 约每 10 秒增加 106--110，符合约 10 Hz `CAR_POSE` 发送；CRC、长度和 payload 丢弃计数不增加，
且 `car_age` 已从 `-1` 变为小于 250 ms，证明 MCU→LoRa→地面站位姿显示链路有效。

## 待修复项

1. **已修复（同日后续回归）**：根因是 UART4 前台轮询被 UART5 LoRa 的阻塞发送打断，触发 ORE 且让
   14 B 帧失步。`bsp_robot_uart.c` 已改为 UART4 RXNE 中断 + 256 B 环形缓冲。
   刷写 `FLASH=30800 B` 后，COM13 在 5.243 s 内由 `legacy_ok=262` 连续增长至 `367`（约 20 Hz），
   `legacy_bad=0, uart_err=0, ring_ovf=0`；地面站统计同时以约 10 Hz 增长且 `car_age<=0.098 s`。
   下载过程中出现的约 11.69 s 接收空档是复位烧录窗口，不是运行态失效。
2. 静止车端已读到原始合法帧 `FA000000060000000800000060AB`，解码为 `X=6 cm,Y=8 cm,yaw=9.6°`。
   因此“坐标很大”不是 MCU 字节序/载荷布局问题；应在 Pi 雷达侧检查其零位定义和静止漂移，MCU 不加入
   未授权坐标偏置。
3. Pi 仍需升级为 `src=0x31,dst=0x32,len=22` 的原生 V2.2 入口，提供真实 `SourceTimeMs`、速度及校准 ID。
4. Pi 校准闭环后，再验收 `CALIBRATED=1`、固定非零 `CalibrationId`、10 Hz 时隙和任务请求。

## 冷启动回归（后续）

一次 MCU 上电后，UART4 端仍为连续健康：`legacy_ok=1235, legacy_bad=0, uart_err=0, ring_ovf=0`，
且 `X=-468 cm,Y=-174 cm,yaw=62.3°` 直接来自 `FA010001D4010000AE0000026FAB`。地面站却保持
`car_age` 过期，原因是旧兼容路径把 MCU boot uptime 当作 `SourceTimeMs`；雷达延迟开始输出后，地面站把
小于上一会话的时间判为 stale。

修复为“首个有效旧帧起的 MCU 转发 epoch”后，重新下载不重启地面站即恢复：

```text
MCU: t=16209 ms, radio_pose_tx=162, legacy_ok=324, bad=0, uart_err=0,
     x=-472 cm, y=-167 cm, source_time=16146 ms, age=19 ms
Ground station: car_age=0.000 s，10 s 内有效帧约 +120
```

本回归证明车端冷启动的 freshness 问题已修复；负的大坐标是 Pi 原始输入，待 Pi 雷达零位核查，不能由
STM32 端私自偏移修正。
