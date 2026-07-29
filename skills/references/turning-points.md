# Durable Decisions

| Decision | Status | Consequence |
| --- | --- | --- |
| Primary local guidance is eight-channel gray tracking | Confirmed | Use primary ADC PC0/PC1/PC2 plus gray IO PG0/PG1 in every competition image |
| Front TB6612 is the motion baseline | Confirmed | Preserve PA2/PA3, PE2-PE6, TIM5 and TIM3 ownership |
| Four-wheel rear TB6612 mapping uses PF1-PF4 directions and PC2 STBY | Bench-only | PC2 cannot coexist with gray ADC 2 |
| Rear PWM moved to TIM1 PE13/PE14 | Bench verified at MCU output level | Requires a rear-STBY remap for competition |
| Rear encoders PB6/PB7 and PC6/PC7 | Wired/test-input stage | Hardware TIM4/TIM8 encoder mode and sign calibration remain pending |
| Car starts tasks from physical buttons | D-task requirement | Do not add ground-station task-start control |
| Car sends pose at 10 Hz as LoRa time base | V2.1 requirement | Do not transmit arbitrary free-running telemetry |
| A-to-B is coordinated but deadline-bound | D-task requirement | Measure and preserve margin below 15 s; do not label it merely “slow” |

Whenever a row changes, update `rules.md`, the project baseline document, and
the relevant build/test evidence in the same change.
