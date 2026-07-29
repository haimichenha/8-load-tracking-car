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
4. `docs/D题_通用通信与接口规范_v2.1.docx` for V2.1 communication behavior.
5. `docs/D题 回复.txt` for official clarifications.
6. Historical MSPM0/lidar, old motor tests, chat notes, and web examples.

When hardware and code disagree, stop expansion work. Record the physical pin,
the configured peripheral mode, the expected signal, and the observed result
before changing either one.

## Current Facts And Non-Facts

- The eight-channel gray module uses five MCU interfaces: primary ADC
  `PC0/PC1/PC2` plus gray GPIO `PG0/PG1`. The exact eight-position decoding,
  ADC thresholds, and digital polarity require bench calibration.
- Front TB6612 is the proven drive chain: PWM `PA2/PA3`, direction `PE2-PE5`,
  and `PE6` STBY. Front encoders use `TIM5 PA0/PA1` and `TIM3 PA6/PA7`.
- The present rear TB6612 bench definition uses `PE13/PE14` PWM,
  `PF1-PF4` direction, and `PC2` STBY. PC2 conflicts with the gray ADC main
  group, so this is not a competition pin map until STBY is remapped.
- Rear encoder pins are wired as test inputs only. Their TIM4/TIM8 hardware
  encoder initialization and direction calibration are not yet complete.
- LoRa module UART pins, final rear-direction remap, radar serial ownership,
  wheel circumference, and final speed limits remain hardware decisions. Do
  not manufacture a pin assignment or a physical performance claim for them.

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
