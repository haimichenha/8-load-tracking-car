# Radar pose back-up interface — 2026-07-30

## Recorded link information

- Radar-car Pi: `ubuntu@192.168.0.131`; local key:
  `~/.ssh/lidar_humble_ed25519`.
- `/dev/mcu_usb -> /dev/ttyUSB0`, CP2102, is owned by the Pi pose bridge.
- The bridge process is `pose_bridge_minimal_humble.py`, configured for 115200
  baud, 20 Hz output and serial enable.
- At the inspection time, RF2O reported `Waiting for laser_scans...`; the
  bridge therefore reported `valid=0, reason=odom_stale`. A stale packet must
  never be used for motion.

## STM32 back-up path

```text
Pi radar/SLAM pose (X/Y/yaw) --UART4--> validated 14-byte pose parser
```

- UART4 frame: `FA | sx xxx | sy yyy | syaw yyy | AB`, 14 bytes.
- `sx/sy/syaw`: `00=positive`, `01=negative`; X/Y unit cm; yaw unit 0.1°.
- Parser validity requires framing, sign/range checks, UART-error-free receive,
  and age no more than 250 ms.

## Current field controller selection

`LineFollowJY901Debug` does **not** initialize or use this UART4 path for
steering. It uses only the verified direct JY901 link on remapped USART2
(`PD5=MCU TX -> JY901 RX`, `PD6=MCU RX <- JY901 TX`, 9600 8N1). Gray X1--X8
remains the primary line-error source and front encoder LADRC remains the speed
inner loop. Do not combine radar yaw and JY901 yaw until an explicit
source-selection/fusion test has acceptance evidence.
