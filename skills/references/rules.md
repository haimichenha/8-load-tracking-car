---
name: stm32f103zet6-d-mission-rules
description: Current STM32F103ZET6 D-task car pin ownership and mode guards. Read before changing hardware-related code.
---

# Pin And Mode Rules

## 1. Front Drive: Competition Baseline

| Function | Pin | Peripheral / rule |
| --- | --- | --- |
| Front-left PWM | PA2 | TIM2_CH3, 20 kHz |
| Front-right PWM | PA3 | TIM2_CH4, 20 kHz |
| Front-left AIN1 / AIN2 | PE2 / PE3 | GPIO output |
| Front-right BIN1 / BIN2 | PE4 / PE5 | GPIO output |
| Front TB6612 STBY | PE6 | GPIO output; low is the safe default |
| Front-left encoder A / B | PA0 / PA1 | TIM5_CH1 / CH2 encoder interface |
| Front-right encoder A / B | PA6 / PA7 | TIM3_CH1 / CH2 encoder interface |

`PA2/PA3` are motor PWM, never USART2 default pins. If gyro UART is used,
use USART2 remap `PD5/PD6` only after verifying the board wiring.

## 2. Eight-Channel Gray Tracking: Competition Baseline

The eight-channel gray module uses five MCU interfaces, not eight direct GPIO
inputs. Keep this ownership in every competition image:

| Interface | Pin | Role |
| --- | --- | --- |
| Gray ADC 0 / 1 / 2 | PC0 / PC1 / PC2 | Primary three-channel ADC group |
| Gray IO 0 / 1 | PG0 / PG1 | Digital gray module interface |
| ADC reserve | PC3 / PB0 / PB1 | Do not use unless the primary group is replaced and documented |
| Analog/DAC reserve | PA4 / PA5 | Not gray-tracking inputs |

The module's conversion from these five signals to the eight physical gray
positions is a hardware contract that must be recorded before controller work.
Measure black/white levels, ADC thresholds, sensor order, error sign, and
lost-line behavior on the actual tape. Do not assume the old PF-based
`bsp_ir_gpio` driver represents this wiring; it is a historical implementation.

## 3. Rear TB6612: Bench-Only Mapping

| Function | Pin | Status |
| --- | --- | --- |
| Rear PWMA / PWMB | PE13 / PE14 | TIM1_CH3 / CH4, full remap; bench only |
| Rear AIN1 / AIN2 | PF1 / PF2 | GPIO output; no current gray conflict |
| Rear BIN1 / BIN2 | PF3 / PF4 | GPIO output; no current gray conflict |
| Rear STBY | PC2 | Conflicts with primary gray ADC 2 |
| Rear-left encoder A / B | PB6 / PB7 | Input polling only; TIM4 encoder mode pending |
| Rear-right encoder A / B | PC6 / PC7 | Input polling only; TIM8 encoder mode pending |

`FourWheelTb6612Debug` is a lifted-wheel qualification image. It must not
initialize the gray ADC interface. Conversely, a line-following mission image
must not initialize rear STBY on PC2, and its safety module must leave PC2 in
the gray ADC configuration.

Before a four-wheel competition image exists, physically remap rear STBY away
from PC2, update `bsp_aux_tb6612.*` and the motor-safe defaults, then perform
direction, encoder, and gray-array regression tests. Do not select the
replacement pin from this document by guess.

## 4. Serial And Communication Ownership

| Link | Current ownership | Constraint |
| --- | --- | --- |
| USART1 PA9/PA10 | Diagnostic log, 115200 8N1 | Preserve for bring-up and test evidence |
| USART2 PD5/PD6 remap | Gyro candidate | Do not use PA2/PA3 |
| UART4 PC10/PC11 | Radar candidate | Verify actual module wiring before enabling |
| UART5 PC12/PD2 | Pi/wireless candidate | Verify actual module wiring before enabling |
| USART3 PD8/PD9 remap | Conditional expansion | Keep free until LoRa/other module ownership is confirmed |

The D-task LoRa protocol is mandatory, but this table does not assert a final
LoRa UART. Assign it only after the actual module and all competing interfaces
have been checked.

## 5. Timer Ownership

- TIM2: front PWM. Do not re-purpose its period during a motor run.
- TIM3 and TIM5: front encoders. Do not generate regular PWM from a timer in
  encoder mode.
- TIM1: rear PWM only in `BENCH_FOUR_WHEEL`; enable full remap and advanced
  timer main output when that bench image is used.
- TIM4 and TIM8: reserved for the planned rear encoder pairs. Do not claim
  closed-loop four-wheel control until these are configured and calibrated.
- TIM6/TIM7: internal scheduling candidates, no external PWM channels.

## 6. Wiring And Safe-State Rules

1. Connect every motor supply, TB6612 logic ground, encoder ground, and MCU
   ground together before testing PWM.
2. Keep both STBY pins low at reset, error, radio timeout handling, and mode
   change. Enable only after direction and PWM compare values are set.
3. Treat a changed PWM register as MCU evidence only. Verify the motor supply,
   TB6612 VM/VCC/GND, output terminals, wheel direction, and encoder response
   separately.
4. Change only one of wiring, remap, direction sign, PWM period, or controller
   gain per test round.
