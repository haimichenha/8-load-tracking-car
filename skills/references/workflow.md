---
name: stm32f103zet6-d-mission-workflow
description: Bench-to-field validation workflow for the STM32F103ZET6 D-task gray-line-following car.
---

# Bring-Up And Validation Workflow

## 1. Select The Mode

Write `BENCH_FOUR_WHEEL` or `COMPETITION_LINE_FOLLOW` in the test record. Do
not flash a bench image onto a vehicle expected to use the gray array.

## Flash The Current Competition Image

Use the verified SWD path from the `lidar_car` project root after removing
payload risk or lifting the wheels:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File "scripts\jlink_flash.ps1" -Configuration LineFollowMissionDebug -SwdSpeedKhz 50
```

This command configures, builds, downloads, compares every programmed section,
then resets and starts `LineFollowMissionDebug`. Accept the flash only when the
command exits `0` and every `compare-sections` range reports `matched`.

Use `scripts\flash_line_follow_uart.cmd COMx` only as a ROM-bootloader fallback;
it requires `BOOT0=1`, reset during connection, then `BOOT0=0` before running
the application. A missing Windows diagnostic COM port never invalidates an
otherwise successful SWD section comparison.

## 2. Bench Four-Wheel Qualification

1. Lift all wheels. Verify motor supply, 3.3 V logic, common ground, and both
   STBY lines low before reset.
2. Build and flash `FourWheelTb6612Debug`.
3. Confirm its bounded run: one-second wait, 65% PWM for four seconds, then
   coast and disable. Use `G`, `B`, and `S` only in a safe test area.
4. Record each wheel's direction and whether its encoder input changes. A
   static encoder count means no closed-loop conclusion.
5. Stop after hardware qualification. Do not add gray tracking, LoRa, or
   mission code to this image.

## 3. Competition Pin Gate

Complete all items before enabling the D-task state machine:

1. Configure PC0/PC1/PC2 as `AD0/AD1/AD2` GPIO address outputs and PG0 as
   selected digital `OUT`; PG1 is unused. With all eight heads on white, latch
   the all-white mask under actual field lighting.
2. Verify X1--X8 by scanning each 3-bit address and moving the selected
   physical position between adjacent white and the real black line. Record
   the switch, polarity, and error sign; do not use historical ADC values.
3. If four-wheel drive is required, preserve rear STBY on `PB9`, update code
   and `rules.md` if its wiring changes, then rerun the complete gray test.
4. Confirm front motor direction, encoder sign, and the safe stop path.
5. Do not use the presence of a passing bench log as proof that this gate has
   passed.

## 4. Gray Tracking Bring-Up

1. Before a physical-position mapping exists, print raw eight-channel mask,
   all-white-relative active mask, and stable mask only. Add decoded L1-L8
   states, weighted error, line-lost state, target speed, left/right targets,
   and output limits only after the mapping is verified.
2. Verify sensor order with per-position map-white/map-black pairs. Do not
   tune PID until the available inputs produce repeatable contrast and map to
   physical positions; ADC raw values alone are not a decode.
3. Establish a low-speed straight-line baseline, then gentle curves, then the
   tightest turn on the field. Tune the sensor threshold/polarity and steering
   sign before increasing speed.
4. Use encoder speed feedback as the inner loop. Apply gyro yaw correction as
   a bounded correction; gray error remains the path-following reference.
5. Define an explicit lost-line action with bounded speed and timeout. Never
   use a radio ACK or ground-station command as line-following control.

## 5. D-Task Coordination

1. At the physical car button, create one task request and start the car.
2. Measure A-to-B path time. Set its target so worst-case measured time remains
   below 15 s with margin; coordination may be slower than other sections but
   cannot violate the deadline.
3. After B, use separately tested faster limits only where gray tracking,
   stopping distance, and payload/platform stability permit them.
4. Keep the vehicle following the complete line loop while the air vehicle
   handles its own mission. Do not stop the car after a wireless ACK.
5. Run the LoRa timing from fresh `CAR_POSE` receipt/transmit timing, not from
   a free-running debug timer. See `d-mission-car.md`.

## 6. Required Evidence Per Stage

| Stage | Minimum evidence |
| --- | --- |
| Pin change | Updated table, build preset, and a signal or observation |
| Motor direction | Wheel, command sign, physical direction, encoder sign |
| Gray tracking | L1-L8 mapping, polarity, error sign, lost-line behavior |
| A-to-B | Distance/course version, repeated time measurements, selected limit |
| LoRa | Frame type, sequence, CRC result, timestamp/age, slot timing |
| Mission button | Task type, mission ID, retry count, no duplicate trigger |

On any unsafe, stale, or contradictory result: stop the motor outputs, retain
the log, and return to the earliest failed stage.
