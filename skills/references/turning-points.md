# Durable Decisions

| Decision | Status | Consequence |
| --- | --- | --- |
| Primary local guidance is gray tracking | Confirmed | Use mux address outputs AD0/AD1/AD2 on PC0/PC1/PC2 plus selected OUT on PG0 in every competition image; X1-X8 mapping remains incomplete |
| Gray interface correction | Confirmed from YB-MVX05-V1.0 board/silkscreen | PC0/PC1/PC2 are not ADC inputs; they select the module's 74HC4051-style X1-X8 output. Historical ADC logs are invalid mapping evidence |
| Front TB6612 is the motion baseline | Confirmed | Preserve PA2/PA3, PE2-PE6, TIM5 and TIM3 ownership |
| Four-wheel rear TB6612 mapping uses PF1-PF4 directions and PB9 STBY | Bench-only | No gray pin conflict; four-wheel competition regression remains pending |
| Rear PWM moved to TIM1 PE13/PE14 | Bench verified at MCU output level | Requires a rear-STBY remap for competition |
| Rear encoders PB6/PB7 and PC6/PC7 | Wired/test-input stage | Hardware TIM4/TIM8 encoder mode and sign calibration remain pending |
| Car starts tasks from physical buttons | D-task requirement | Do not add ground-station task-start control |
| Car sends pose at 10 Hz as LoRa time base | V2.2 requirement | Do not transmit arbitrary free-running telemetry |
| MaixCam session ownership | V2.2 requirement | Flight controller creates ModeSeq and controls camera; car only preserves TaskType/MissionId and keeps following the line |
| A-to-B is coordinated but deadline-bound | D-task requirement | Measure and preserve margin below 15 s; do not label it merely “slow” |

Whenever a row changes, update `rules.md`, the project baseline document, and
the relevant build/test evidence in the same change.
