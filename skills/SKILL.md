---
name: stm32f103zet6-d-mission-car
description: "Use when working on the STM32F103ZET6 land-and-air electronic-competition car for D task: eight-channel gray line tracking, four-wheel TB6612 bring-up, encoders, gyro/radar pose integration, LoRa V2.1 communication, A-to-B speed coordination, physical start buttons, staged testing, or related documentation. Read before changing car hardware code, mission behavior, pin maps, or test records."
---

# D Task Car Skill

Use this skill for the car-side STM32F103ZET6 project. The competition vehicle
is a line-following vehicle: the primary local guidance sensor is the
eight-channel gray tracking array, not a camera or an assumed lidar-only path.

## Read In This Order

1. Read `references/rules.md` before changing GPIO, TIM, PWM, encoder, UART,
   motor, or gray-tracking code.
2. Read `references/d-mission-car.md` before changing LoRa, Pi pose input,
   task buttons, state-machine behavior, coordination speed, or D-task logic.
3. Read `references/workflow.md` before wiring, flashing, bench testing, field
   testing, or interpreting logs.
4. Read `references/core.md` and `references/docs.md` when resolving source
   conflicts or updating project documentation.
5. Read `references/radar_bigturn_good_baseline_20260515.md` only for
   historical radar tuning; it never defines the current pin map or protocol.

## Mandatory Decisions

State the active mode before editing or flashing:

- `BENCH_FOUR_WHEEL`: lifted-wheel TB6612 qualification. PC2 is rear STBY and
  the gray ADC interface is intentionally disabled.
- `COMPETITION_LINE_FOLLOW`: the gray module owns PC0/PC1/PC2 plus PG0/PG1.
  The present rear STBY on PC2 is forbidden until it is physically remapped
  and the code is updated.

Never claim both modes are enabled by the same PC2 configuration. Resolve the
physical remap, code macros, safety defaults, and documentation together.

## Operating Rules

1. Keep the car moving after a valid task request; LoRa ACKs, telemetry loss,
   and ground-station display state do not independently stop line following.
2. Start a D-task request only from the physical button on the car. The ground
   station is display and preflight-calibration only during the task.
3. Use encoder speed feedback and gyro yaw correction as assistance around the
   gray tracking controller. Do not replace the gray array with inferred pose
   or radio commands.
4. Choose A-to-B speed from measured path time with margin below the required
   15 s. Other verified sections may be faster, but only after the line,
   encoder, and braking behavior have been tested at that speed.
5. Treat `docs/D题_通用通信与接口规范_v2.1.docx` as the communication
   authority. Do not invent a LoRa frame, free-running transmit schedule, or
   task acceptance semantic.
6. Keep bench evidence separate from competition evidence. PWM register values
   show MCU output, not motor torque, wheel direction, or closed-loop quality.

## Reference Map

| Reference | Purpose |
| --- | --- |
| `references/rules.md` | Current pins, peripheral ownership, and the gray/rear-driver conflict |
| `references/d-mission-car.md` | D-task car responsibilities, coordinate contract, LoRa timing, speed policy |
| `references/workflow.md` | Bench-to-field bring-up and acceptance order |
| `references/core.md` | Source priority, evidence rules, and documentation updates |
| `references/docs.md` | Authoritative local documents and code locations |
| `references/turning-points.md` | Durable hardware and mission decisions |

Keep the entry concise. Put pin facts in `rules.md`, D-task protocol details in
`d-mission-car.md`, and test evidence in project logs rather than this file.
