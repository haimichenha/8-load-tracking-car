---
name: stm32f103zet6-d-mission-workflow
description: Bench-to-field validation workflow for the STM32F103ZET6 D-task gray-line-following car.
---

# Bring-Up And Validation Workflow

## 1. Select The Mode

Write `BENCH_FOUR_WHEEL` or `COMPETITION_LINE_FOLLOW` in the test record. Do
not flash a bench image onto a vehicle expected to use the gray array.

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

1. Configure PC0/PC1/PC2 as the primary gray ADC group and PG0/PG1 as gray
   digital inputs; record raw values under the actual tape and lighting.
2. Verify the module's reconstruction of L1-L8 by covering each physical
   position in turn. Record threshold, polarity, and error sign.
3. If four-wheel drive is required, remap rear STBY away from `PC2`, update
   code and `rules.md`, then rerun the complete gray test.
4. Confirm front motor direction, encoder sign, and the safe stop path.
5. Do not use the presence of a passing bench log as proof that this gate has
   passed.

## 4. Gray Tracking Bring-Up

1. Print raw ADC0-ADC2, gray IO0/IO1, decoded L1-L8 states, weighted error,
   line-lost state, target speed, left/right targets, and output limits.
2. Verify sensor order by covering one physical gray position at a time. Do
   not tune PID until the five-input decode maps to every physical position.
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
