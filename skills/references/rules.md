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

The Yahboom module uses a 74HC4051-style address selector: it has four MCU
interfaces, not eight direct GPIO inputs. Keep this ownership in every
competition image:

| Interface | Pin | Role |
| --- | --- | --- |
| Gray AD0 / AD1 / AD2 | PC0 / PC1 / PC2 | GPIO outputs; address bit0 / bit1 / bit2 selecting X1--X8 |
| Gray OUT | PG0 | GPIO input; digital value of the selected head |
| PG1 | Unused | Do not claim it as a gray input without physical wiring and a new test |
| ADC reserve | PC3 / PB0 / PB1 | Do not use unless the primary group is replaced and documented |
| Analog/DAC reserve | PA4 / PA5 | Not gray-tracking inputs |

Address `000`--`111` selects X1--X8 respectively. The 2026-07-30 mux8 evidence
log (`lidar_car/logs/gray_mux_retest_20260730-104601.log`) verifies white
`0x00` and black-line activation `X1..X8 = 0x01..0x80`, one bit per head.
On the current map, a normally centered black line is `X4|X5 = 0x18`
(`lidar_car/logs/gray_mux_center_x4_x5_20260730-110625.log`); this is the
zero point. Vehicle-left and vehicle-right probes verified `X4 = 0x08` and
`X5 = 0x10` respectively; use weights `-7/-5/-3/-1/+1/+3/+5/+7` for static
diagnostics. This sign is not yet wired to motor control.
Use `raw_mask XOR all_white_mask` to remove the module's output polarity.
The remaining gray-sensor check is the physical left/right error sign and
lost-line behavior on the actual tape. Do not use the old ADC test or old PF-based
`bsp_ir_gpio` driver; both are historical implementations.

## 3. Rear TB6612: Bench-Only Mapping

| Function | Pin | Status |
| --- | --- | --- |
| Rear PWMA / PWMB | PE13 / PE14 | TIM1_CH3 / CH4, full remap; bench only |
| Rear AIN1 / AIN2 | PF1 / PF2 | GPIO output; no current gray conflict |
| Rear BIN1 / BIN2 | PF3 / PF4 | GPIO output; no current gray conflict |
| Rear STBY | PB9 | GPIO output; no current gray conflict |
| Rear-left encoder A / B | PB6 / PB7 | Input polling only; TIM4 encoder mode pending |
| Rear-right encoder A / B | PC6 / PC7 | Input polling only; TIM8 encoder mode pending |

`FourWheelTb6612Debug` is a lifted-wheel qualification image. It must not
initialize the gray address-scan interface. A line-following mission image may use the
PB9 rear STBY only after four-wheel motion and gray-array regression evidence
exists; it must still keep all motor outputs in the safe state on fault.

Before a four-wheel competition image exists, preserve the verified PB9 rear
STBY wiring, update `bsp_aux_tb6612.*` and motor-safe defaults with any future
change, then perform direction, encoder, and gray-array regression tests.

## 4. Serial And Communication Ownership

| Link | Current ownership | Constraint |
| --- | --- | --- |
| USART1 PA9/PA10 | Diagnostic log, 115200 8N1 | Preserve for bring-up and test evidence |
| USART2 PD5/PD6 remap | Current line-follow JY901 yaw source, 9600 8N1 | `PD5=MCU TX -> JY901 RX`; `PD6=MCU RX <- JY901 TX`; do not use PA2/PA3 |
| UART4 PC10/PC11 | Pi radar/SLAM pose ingress | PC10=MCU TX reserved; PC11=MCU RX <- Pi `/dev/mcu_usb` TX; 115200 8N1 |
| UART5 PC12/PD2 | LoRa transparent wireless | Bench verified 2026-07-30: PC12=MCU TX -> radio RX; PD2=MCU RX <- radio TX; 115200 8N1 |
| USART3 PD8/PD9 remap | Conditional expansion | Keep free until LoRa/other module ownership is confirmed |

The D-task LoRa transport is assigned to UART5 by the 2026-07-30 bidirectional
fixture test. Do not concurrently attach a Pi/Nano to PC12/PD2. The Pi pose
bridge is instead assigned to UART4. Its current field frame is the 14-byte
`FA | sign+x[3] | sign+y[3] | sign+yaw10[3] | AB` packet: X/Y are integer cm,
yaw is 0.1 degree, signs are `00/01`, and the Pi transmits only fresh accepted
poses. The STM32 must reject malformed frames and treat age over 250 ms as
stale. This local bridge packet is not the LoRa V2.2 packet.

Current radar-car SSH endpoint, recorded on 2026-07-30: `ubuntu@192.168.0.131`
using the local `lidar_humble_ed25519` key. It is a DHCP field address: verify
reachability and host key before every remote operation; do not hard-code it
into MCU firmware.

For the current `LineFollowJY901Debug` field image, only the direct JY901
USART2 yaw source participates in heading correction. UART4 radar pose parsing
is retained as a separately validated back-up interface and must not be mixed
with JY901 yaw in one controller without a source-selection/fusion test.

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
5. Competition line-follow start/stop key is **KEY2 = PG10**, active low with
   a pull-up. `bsp_key2.*` names PC4 in historical code and must not be reused
   as evidence for the current physical PG10 start key.
