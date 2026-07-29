# Document Map

## D-Task Authority

| Path | Use |
| --- | --- |
| `F:\keil5\stm\ZET6 HAl\lidar_car\docs\D题_通用通信与接口规范_v2.1.docx` | V2.1 frame layout, coordinate contract, LoRa slots, reliability, role boundaries |
| `F:\keil5\stm\ZET6 HAl\lidar_car\docs\D题 回复.txt` | Official clarifications: physical car button, 15 s A-to-B, line-following freedom, communication permission, platform limit |
| `F:\keil5\stm\ZET6 HAl\lidar_car\docs\D题_Agent修改提示词_v2.1.md` | Role-specific implementation prompts; use B.3 for car-side scope |
| `F:\keil5\stm\ZET6 HAl\lidar_car\docs\D题小车端实施基线.md` | Current car-side design decisions and unresolved hardware gates |

## Current Hardware And Test Sources

| Path | Use |
| --- | --- |
| `Hardware/bsp_motor.h` and `Hardware/bsp_motor.c` | Front TB6612 pins, PWM, direction, and STBY behavior |
| `Hardware/bsp_ir_gpio.h` and `Hardware/bsp_ir_gpio.c` | Historical PF direct-GPIO implementation; do not use as the current five-pin gray wiring source |
| `Hardware/bsp_aux_tb6612.h` and `Hardware/bsp_aux_tb6612.c` | Rear TB6612 bench-only mapping; inspect before any four-wheel test |
| `Hardware/app_four_wheel_tb6612_test.c` | Bounded four-wheel bench test behavior |
| `logs/four_wheel_tb6612_wiring_20260729.md` | Bench wiring record; not a competition wiring approval |

## Update Policy

Update `D题小车端实施基线.md` and `references/rules.md` together when a
physical pin, selected interface, safety gate, or mission behavior changes.
Keep the Word communication specification unchanged unless the team formally
revises the protocol. Do not treat historical test logs as a replacement for
the current pin table.
