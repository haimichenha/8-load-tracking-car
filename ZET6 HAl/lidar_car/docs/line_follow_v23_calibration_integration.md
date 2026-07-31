# V2.3 Calibration Integration

This page is the current car-side integration checklist. The historical
`radar_pose_line_follow_switch_20260730.md` describes the migration from the
14-byte radar stream; it does not replace the V2.3 calibration transaction.

> 当前文中 MCU-local 路径是针对单向 `Pi TX -> MCU RX` 物理现状的**兼容联调**。
> 发布 V2.3 要求 Pi 承担平台中心换算与校准/复位 ACK；二者的差距、几何验证和
> 计分限制见 [D题_赛题合规差距清单_20260731.md](D题_赛题合规差距清单_20260731.md)。
> 未关闭这些门槛前，本页的成功条件只能证明车端/无线互操作，不能证明正式得分。

## Current MCU behavior

| Item | Implemented car behavior |
| --- | --- |
| `0x80 CAR_POSE` | Pi sends raw `0x31 -> 0x32` only; MCU retains it unchanged internally and publishes `0x30 -> 0x10` at 10 Hz. The LoRa egress mapping is `X_lora=-(X_pi-13 cm)+DeltaX`, `Y_lora=-Y_pi+DeltaY`, `yaw_lora=yaw_pi`; local `CALIBRATED/CalibrationId` is written after that mapping. |
| `0x83 CALIBRATION_SET` | Ground station sends `0x40 -> 0x30`, `ACK_REQUIRED`, 12-byte payload. MCU accepts it only while the car is not running, commits `DeltaX/DeltaY/CalibrationId` to MCU RAM, and replies directly to ground in the next car slot. It does not transmit to or wait for Pi. A new request sequence may replace an existing local calibration for debugging; the received Delta is always the complete offset from raw Pi pose, never an increment. Repeated `Src+Type+Seq` requests are deduplicated for 5 seconds. |
| `0x85 MAINTENANCE_RESET` | A stopped-car PG12 short press after 50 ms debounce clears the MCU-local Delta, `CALIBRATED`, and CalibrationId, then broadcasts three LoRa copies directly. Pi is not queried. The firmware has no long-press compatibility branch. |
| Local fallback | PG13/PG9 require the A marker and fresh JY901 only. Missing, stale, or implausible Pi/radar data records `task_coordination_deferred` while gray/JY901/encoder line following continues. Three valid raw pose samples plus MCU-local calibration may establish `0x81` later without restarting or stopping the car. |
| Task 1 speed | PG13 uses a 130 mm/s cooperative straight-line target. Existing curve, wide-line, and lost-line caps remain in force. |
| Task 2 speed | PG9 uses a 150 mm/s cooperative straight-line target. Existing curve, wide-line, and lost-line caps remain in force. |

## PG12 Standard Boundary

`LineFollowMissionDebug` implements only the short-press procedure. It must not
be described as requiring 12 s stationary or a long hold. The authoritative
V2.3 Appendix timing procedure remains an external scoring requirement and does
not close the separate Pi ownership/ACK gap documented in G3; it is not a CMake
option or a second physical button behavior in this firmware.

## Physical topology and ownership

The installed Pi link is one-way: `Pi TX -> MCU UART4 PC11 RX`. Pi sends raw pose
data only. It does not receive MCU messages, store `DeltaX/DeltaY/CalibrationId`,
or return a control ACK. The MCU is the calibration owner:

`Ground 0x83 -> MCU local RAM -> MCU ground ACK`

`Pi raw pose -> MCU raw internal state -> MCU calibrated LoRa CAR_POSE`

The Pi input contract is therefore limited to a fresh raw `CAR_POSE(0x80)` at 10 Hz:

1. Send `Src=0x31,Dst=0x32` with valid CRC, `FIELD_GLOBAL`,
   `POSITION_VALID|YAW_VALID`, raw X/Y/yaw, and a monotonic source time.
2. Keep `CALIBRATED=0` and `CalibrationId=0`. An upstream legacy mark is not
   trusted or echoed as MCU calibration.
3. Do not apply the center offset, mirror mapping, or site Delta in this
   compatibility input stream. MCU egress computes
   `X_lora=-(X_pi-13 cm)+DeltaX` and `Y_lora=-Y_pi+DeltaY`; yaw is
   relayed unchanged as `yaw_lora=yaw_pi`.

On a valid stopped-car `0x83`, the MCU validates source/destination, ACK flag,
12-byte payload, nonzero CalibrationId, `APPLY=0x01`, and reserved byte. It stores
the three calibration values in RAM, queues `ACK_ACCEPTED` directly to the ground
station, and leaves raw Pi/radar state untouched. It deduplicates the same
`Src+Type+Seq` for five seconds. A stopped car may accept a new request
sequence and replace an existing local calibration. For a changed offset, use
a new CalibrationId; do not add the new Delta to the previous one.

The observed legacy `FA ... AB` stream remains an allowed raw migration input. It
cannot ACK anything, and no ACK is required from it. It can produce a calibrated outgoing
`CAR_POSE` only after the MCU has accepted `0x83`; this still does not block a
local lap when Pi data is absent.

## Field Test

1. Flash the current `LineFollowMissionDebug` image and leave the car stopped.
2. Confirm the ground station receives a fresh uncalibrated `CAR_POSE`
   (`CALIBRATED=0, CalibrationId=0`) and a current `FLIGHT_TELEMETRY` in IDLE.
3. Run the dry run first:

```bat
cd /d "F:\keil5\stm\ZET6 HAl\lidar_car"
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\test_line_follow_calibration_link.ps1 -WirelessPort COMx -DiagPort COMy
```

4. With the car still stationary and raw Pi pose available, record the last
   uncalibrated outgoing X/Y. The script waits for `CAR_POSE.Seq mod 5 = 2`,
   delays 35 ms, then sends `0x83` inside the required maintenance window:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\test_line_follow_calibration_link.ps1 -WirelessPort COMx -DiagPort COMy -Send
```

5. `WirelessPort` is the ground-side LoRa serial port and `DiagPort` is the
   MCU diagnostic serial port; they must be different. Use `-DiagPort ""` if
   the diagnostic port is unavailable. The script creates its own `logs`
   directory when needed.
6. Pass criteria: an accepted `0x83` ACK from `0x30 -> 0x40` without any Pi
   ACK, followed by at least three `0x80 CAR_POSE` frames with `CALIBRATED=1`
   and the requested CalibrationId. Relative to the recorded uncalibrated output,
   `X_after-X_before=DeltaX` and `Y_after-Y_before=DeltaY`. Raw Pi/radar
   diagnostic X/Y must remain unchanged by the transaction.
7. Query diagnostics and confirm `pg12_short_press_only=1`, `motors_enabled=0`,
   and `maint_broadcast_left=0`.
   Low-active short-press PG12; after the 50 ms debounce the MCU must log
   `maintenance_reset_local_applied,reason=PG12_SHORT_PRESS`, clear local
   calibration without contacting Pi, then emit exactly three `0x85` copies
   with the same ResetId, Seq, and payload while only the final two set
   `RETRANSMISSION`. Subsequent outgoing `0x80` frames return to
   `CALIBRATED=0, CalibrationId=0`.
8. After `maint_broadcast_left=0`, short-press PG12 once more and confirm a new
   ResetId. The ground station then sends a new `0x83` Seq and CalibrationId to
   apply the next offset. Do not press PG12 during motion or during the
   three-frame broadcast.
9. A later short press must increment `maint_reset_attempts` and
   `maint_reset_successes`, use a new ResetId after the previous broadcast has
   completed, and leave `maint_last_result=PG12_SHORT_PRESS`. The V2.3
   Appendix physical-button timing is not implemented by this firmware and
   remains separate from the missing Pi reset ACK chain.

### Recalibration Debug

With the car stopped, a second `0x83` may replace the current MCU-local
calibration without PG12. Supply a new request sequence so the 5 s duplicate
cache does not replay the old ACK. The new `DeltaX/DeltaY` is absolute relative
to raw Pi output; a changed offset should use a new `CalibrationId`. For an
exact before/after delta assertion, provide the old complete offsets to the
test script:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\test_line_follow_calibration_link.ps1 -WirelessPort COMx -DiagPort COMy -DeltaXCm <newX> -DeltaYCm <newY> -PreviousDeltaXCm <oldX> -PreviousDeltaYCm <oldY> -CalibrationId <newId> -Sequence <newSeq> -AllowRecalibration -Send
```
