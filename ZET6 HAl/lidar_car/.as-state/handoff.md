# Electrical Contest Integration Handoff

## Active scope

Prepare the STM32 car firmware and its local documentation for the electrical
contest car-aircraft integration.  The authoritative requirement sources are
the V2.3 protocol DOCX, `docs/\u4efb\u52a1\u8981\u6c42.png`, and `docs/D\u9898 \u56de\u590d.txt`.

## Known implementation evidence

- PG12 is the maintenance-reset key.  PG13 starts task 1 and PG9 starts task
  2; a second press of either task key requests the local emergency stop.
- The car MCU parses V2.3 mission status (`0x82`) and flight telemetry
  (`0x02`). A task session now requires three successfully sent, current-
  calibration `0x80 CAR_POSE` frames before its three-slot `0x81` retry
  sequence; UART5 RXNE buffering protects the air ACK window. MCU-local
  calibration (`0x83`) and maintenance reset (`0x85`) do not need a Pi return
  channel.
- Pi/radar input remains raw inside the MCU. Only outgoing LoRa `0x80 CAR_POSE`
  applies `X_lora=-(X_pi-13 cm)+DeltaX`, `Y_lora=-Y_pi+DeltaY`, and
  `yaw_lora=yaw_pi`. This mapping is never fed back into line following, JY901,
  encoder, or raw radar state.
- Current cooperative target speeds are 130 mm/s for task 1 and 150 mm/s for
  task 2. Task 2 alone can use up to 55% positive PWM (task 1 remains 45%) to
  provide B-point margin when the LADRC output is saturated.
- The default flash route is J-Link SWD at 50 kHz through
  `scripts/jlink_flash.ps1 -Configuration LineFollowMissionDebug -SwdSpeedKhz 50`.
  On 2026-08-01 it connected at 3.30 V, programmed and compare-verified every
  image section, reset the MCU, and COM13 `P` confirmed the running image.
- Radar coordinates are auxiliary progress/preparation evidence. Gray line,
  JY901 heading, and encoder distance remain the motion and stop authorities;
  the car may run a local lap with no radar/Pi input.
- The confirmed field policy is: PG13/PG9 do not wait for Pi, radar, or a
  CalibrationId. If fresh raw pose and MCU-local calibration become available
  while the car is already following the line, the MCU may then establish
  `0x81` cooperation; otherwise it continues locally. The protocol forbids
  fabricating a zero or stale CalibrationId solely to start aircraft
  coordination.
- A stopped car can replace its local calibration for debugging with a new
  `0x83` request sequence. The new Delta is complete relative to raw Pi pose,
  not additive. The current image was physically flashed and boot-checked on
  2026-08-01. The user also reports a successful car-aircraft joint test;
  preserve the next radio capture before treating this as scored evidence.

## Open scoring risks

- The current firmware audit must enforce a 90-second lap policy and record
  the task-2 B-point 15-second milestone. Local vehicle motion must remain
  available without radar/Pi; a scored aircraft session still needs a valid
  calibrated pose and the corresponding wireless task evidence.
- Task 2 must remain at cooperative speed through platform landing and its
  five-second dwell; only the post-platform-takeoff aircraft stage may unlock
  the fast envelope.
- Pi raw-pose deployment, aircraft firmware, air-radio forwarding,
  payload/landing behavior, and graphical ground station are external owners.
  Their absence is a field-test blocker, not a result that MCU code can claim.
- The one-way Pi-to-MCU path is a compatibility field-test topology. It does
  not close the published V2.3 Pi calibration/ACK transaction or prove the
  physical platform-center requirement; do not record a strict scoring pass
  without those external evidence gates.

## Historical archive

The earlier L298N wheel-direction work remains in `state.json` and
`learning.json` as historical evidence.  It is not part of the current
contest-readiness acceptance decision.

## Next minimum action

Run one controlled straight-line field case from A with PG13 or PG9, then
capture `CALIBRATED 0x80 x3 -> 0x81 x3 -> air 0x11 ACK`, `radio_uart_err=0`,
and the frozen line-follow log before changing base speed, PWM limits, or turn
parameters. Preserve paired Pi-input/LoRa-output frames proving
`X_lora=-(X_pi-13 cm)+DeltaX`, `Y_lora=-Y_pi+DeltaY`, and `yaw_lora=yaw_pi`.
