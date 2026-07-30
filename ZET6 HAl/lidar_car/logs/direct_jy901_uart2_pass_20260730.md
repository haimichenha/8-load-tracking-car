# Direct JY901 UART2 reception — 2026-07-30

## Wiring and test image

- Test image: `PwmGyroSweepDebug`; all motors remained stopped (`motion=S`).
- USART2 full remap: `PD5 = MCU TX -> JY901 RX`; `PD6 = MCU RX <- JY901 TX`.
- Gyro default: 9600 8N1; diagnostic UART: USART1 115200.

## Live serial evidence

Two `P` status requests on COM13, separated by two seconds:

| Field | First | Second | Delta |
| --- | ---: | ---: | ---: |
| `gyro_frames` | 2383 | 2418 | +35 |
| `angle_frames` | 1198 | 1215 | +17 |
| `raw_bytes` | 30618 | 31067 | +449 |
| `pd6_fall_edges` | 71981 | 73059 | +1078 |

`pd5_fall_edges=0` is expected because PD5 is the MCU transmit pin. The
increasing valid frames and PD6 edges prove that the JY901 TX reaches the STM32
RX pin and the WIT parser accepts angle frames. `rate_frames=0` in this sample;
derive rate from consecutive yaw samples if direct JY901 becomes the selected
line-follow heading source.

## Selected control source

The current `LineFollowJY901Debug` field image uses this verified direct JY901
stream as its only heading-assistance source. It derives yaw rate from
consecutive angle samples because `rate_frames=0` in the tested configuration.
UART4 Pi radar pose parsing remains a separately retained back-up and is not
mixed into this controller.
