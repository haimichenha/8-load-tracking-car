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
4. `F:\keil5\stm\docs\D题_通用通信与接口规范_v2.2.docx` for V2.2 communication behavior.
5. `docs/D题 回复.txt` for official clarifications.
6. Historical MSPM0/lidar, old motor tests, chat notes, and web examples.

When hardware and code disagree, stop expansion work. Record the physical pin,
the configured peripheral mode, the expected signal, and the observed result
before changing either one.

## Current Facts And Non-Facts

- The gray module uses `PC0/PC1/PC2` as GPIO address outputs `AD0/AD1/AD2`
  and `PG0` as selected digital `OUT`; PG1 is unused. It is a 74HC4051-style
  eight-channel mux, not an ADC array. Capture the all-white mask and then
  obtain X1--X8 physical-position evidence before controller work.
- Front TB6612 is the proven drive chain: PWM `PA2/PA3`, direction `PE2-PE5`,
  and `PE6` STBY. Front encoders use `TIM5 PA0/PA1` and `TIM3 PA6/PA7`.
- The present rear TB6612 bench definition uses `PE13/PE14` PWM,
  `PF1-PF4` direction, and `PB9` STBY. It has no current gray conflict, but
  four-wheel competition regression remains incomplete.
- Rear encoder pins are wired as test inputs only. Their TIM4/TIM8 hardware
  encoder initialization and direction calibration are not yet complete.
- LoRa transparent wireless is bench verified on UART5: `PC12=MCU TX -> radio
  RX`, `PD2=MCU RX <- radio TX`, 115200 8N1. The exact 2026-07-30 V2.2
  `CAR_POSE` fixture passed in the radio-to-PC direction and the calibration
  fixture passed in the PC-to-radio direction. This is transport evidence only:
  no 10 Hz pose publication, slot policy, Pi pose ingress, task request, or
  calibration forwarding is enabled yet. Radar serial ownership, wheel
  circumference, final speed limits, and gray lost-line/wide-line policy remain
  hardware decisions. The flight controller owns MaixCam ModeSeq/session
  control; do not manufacture a car-side camera interface.

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
