# Core Constraints

## Scope

This skill governs the STM32F103ZET6 car for the D land-and-air task. Its
responsibilities are safe motion, eight-channel gray line following, local
pose intake from the Raspberry Pi/radar stack, LoRa transmission and task
requests, plus observable test evidence. It does not decide flight control,
camera release, or ground-station mission control.

## Source Priority

Resolve a conflict in this order:

1. Current physical wiring and a repeatable observation from this build.
2. `references/rules.md` and `docs/D题小车端实施基线.md`.
3. The current STM32F103ZET6 source selected by the active CMake preset.
4. `F:\keil5\stm\ZET6 HAl\lidar_car\docs\D题_通用通信与接口规范_v2.3.docx`
   for authoritative V2.3 communication behavior. The on-wire Version field
   remains `0x02`.
5. `docs/D题 回复.txt` for official clarifications.
6. Historical MSPM0/lidar, old motor tests, chat notes, and web examples.

When hardware and code disagree, stop expansion work. Record the physical pin,
the configured peripheral mode, the expected signal, and the observed result
before changing either one.

## Current Facts And Non-Facts

- The gray module uses `PC0/PC1/PC2` as GPIO address outputs `AD0/AD1/AD2`
  and `PG0` as selected digital `OUT`; PG1 is unused. It is a 74HC4051-style
  eight-channel mux, not an ADC array. The full X1--X8 mapping, white/black
  polarity, centered `X4|X5`, and left/right error sign are verified. The
  remaining gray work is lost-line/wide-line policy and motor-actuation tests.
- Front TB6612 is the proven drive chain: PWM `PA2/PA3`, direction `PE2-PE5`,
  and `PE6` STBY. Front encoders use `TIM5 PA0/PA1` and `TIM3 PA6/PA7`.
- The present rear TB6612 bench definition uses `PE13/PE14` PWM,
  `PF1-PF4` direction, and `PB9` STBY. It has no current gray conflict, but
  four-wheel competition regression remains incomplete.
- Rear encoder pins are wired as test inputs only. Their TIM4/TIM8 hardware
  encoder initialization and direction calibration are not yet complete.
- UART5 LoRa transport is `PC12=MCU TX -> radio RX`, `PD2=MCU RX <- radio TX`,
  115200 8N1. `LineFollowMissionDebug` sends fresh `CAR_POSE (0x80)` at 10 Hz,
  sends physical-button `CAR_TASK_REQUEST (0x81)` in three car slots after a
  valid Pi calibration becomes available, receives `MISSION_STATUS (0x82)` and
  `FLIGHT_TELEMETRY (0x02)`, and emits three urgent `MISSION_ABORT (0x84)`
  copies on a task-key safety stop. The flight controller owns MaixCam
  ModeSeq/session control; do not manufacture a car-side camera interface.
- The current competition image is `LineFollowMissionDebug`: PG13 starts task
  one, PG9 starts task two, and either task key is an active-run safety stop.
  PG12 is the stopped-for-12-s, held-for-2-s maintenance key; PG10 is unused.
  A full-black A marker and fresh JY901 gate local propulsion. Pi/radar pose is
  auxiliary: it supplies a valid `CalibrationId`, coordinate display, B/A
  preparation, and delayed task coordination but never blocks local line
  following. Both tasks start at about 120 mm/s; verified flight-stage gates
  may open the approximately 180 mm/s envelope.

## Documentation And Evidence

Update the project baseline when a pin, direction polarity, timer mode, LoRa
rate, sensor polarity, speed limit, or task transition changes. A passing
stage needs a reproducible command, build/preset, wiring statement, and at
least one log, measurement, photo, or direct observation.

Keep these distinctions explicit:

- `planned`: wiring or behavior not physically verified;
- `bench verified`: isolated test only;
- `competition verified`: gray tracking and mission interfaces active on the
  intended field configuration.
