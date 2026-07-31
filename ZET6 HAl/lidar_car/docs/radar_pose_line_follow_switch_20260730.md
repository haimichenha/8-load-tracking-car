# Radar pose auxiliary interface — 2026-07-30 (historical migration evidence)

> 当前任务一/二联调不使用本页的“雷达替代循迹”结论。JY901 yaw/yaw-rate 与编码器
> 路径距离是车辆运动主判据；雷达坐标仅用于协同位姿、B/A 交叉检查和地面站显示。
> 当前按键、阶段、计时和得分判据见
> [D题_V23_联调与计分矩阵.md](D题_V23_联调与计分矩阵.md)。本页是迁移历史，不是现场入口。

## Recorded link information

- Radar-car Pi: `ubuntu@192.168.0.131`; local key:
  `~/.ssh/lidar_humble_ed25519`.
- `/dev/mcu_usb -> /dev/ttyUSB0`, CP2102, is owned by the Pi pose bridge.
- The bridge process is `pose_bridge_minimal_humble.py`, configured for 115200
  baud, 20 Hz output and serial enable.
- At the inspection time, RF2O reported `Waiting for laser_scans...`; the
  bridge therefore reported `valid=0, reason=odom_stale`. A stale packet must
  never be used for motion.

## Historical STM32 migration path

```text
Pi radar/SLAM pose (X/Y/yaw) --UART4--> legacy 14-byte compatibility parser
```

- UART4 frame: `FA | sx xxx | sy yyy | syaw yyy | AB`, 14 bytes.
- `sx/sy/syaw`: `00=positive`, `01=negative`; X/Y unit cm; yaw unit 0.1°.
- Parser validity requires framing, sign/range checks, UART-error-free receive,
  and age no more than 250 ms.

## Current field controller selection

The historical `LineFollowJY901Debug` target does **not** initialize this
UART4 path for steering. The active `LineFollowMissionDebug` target does
initialize UART4 for the Pi V2.3 pose and calibration-control link, but it
uses that data only for cooperation, coordinates, and A/B cross-checks. It
still uses the verified direct JY901 link on remapped USART2
(`PD5=MCU TX -> JY901 RX`, `PD6=MCU RX <- JY901 TX`, 9600 8N1) for yaw and
yaw-rate steering. Gray X1--X8 remains the primary line-error source and front
encoder LADRC remains the speed inner loop. Do not combine radar yaw and JY901
yaw until an explicit source-selection/fusion test has acceptance evidence.
