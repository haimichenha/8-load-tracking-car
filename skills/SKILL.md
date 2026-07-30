---
name: stm32f103zet6-d-mission-car
description: "Use when working on the STM32F103ZET6 land-and-air electronic-competition car for D task: gray line tracking, four-wheel TB6612 bring-up, encoders, gyro/radar pose integration, LoRa V2.2 communication, MaixCam coordination boundaries, A-to-B speed coordination, physical start buttons, staged testing, or related documentation. Read before changing car hardware code, mission behavior, pin maps, or test records."
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

- `BENCH_FOUR_WHEEL`: lifted-wheel TB6612 qualification. The gray interface is
  intentionally disabled even though rear STBY is now on PB9.
- `COMPETITION_LINE_FOLLOW`: the gray module owns `PC0/PC1/PC2` as
  `AD0/AD1/AD2` address outputs and `PG0` as the selected digital `OUT`
  input. PG1 is unused. Scan and map all eight heads before controller work.

Never claim the unresolved ADC channels form an eight-position decode. Resolve
physical output mapping, thresholds, safety defaults, and documentation together.

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
5. Treat `F:\keil5\stm\docs\D题_通用通信与接口规范_v2.2.docx` as the communication
   authority. Do not invent a LoRa frame, free-running transmit schedule, or
   task acceptance semantic.
6. Keep the MaixCam V2.2 session on the air side. The car provides
   `TaskType/MissionId` and pose; only the flight controller creates `ModeSeq`
   and controls CAMERA_MODE/ACTION. Do not route camera frames through the car.
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
