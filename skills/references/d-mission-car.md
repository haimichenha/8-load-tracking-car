# D-Task Car Mission Contract

## 1. Car-Side Role

The car is responsible for physical start buttons, gray line following, local
motion safety, Pi pose intake, LoRa V2.3 transmission, calibration forwarding,
and task-request retries. The eight-channel gray module is acquired through
GPIO address outputs `AD0/AD1/AD2=PC0/PC1/PC2` plus selected digital
`OUT=PG0`; its full scan must be calibrated. The
Raspberry Pi converts radar/SLAM coordinates to the platform-center
`FIELD_GLOBAL` pose. The car MCU sends that pose over LoRa. The flight
controller alone accepts or rejects flight tasks.

During either mission, the car continues its line-following loop. A wireless
ACK, a telemetry timeout, or a ground-station UI event is not a stop command
for vehicle motion.

## 2. Coordinate And Pose Contract

- Reference point: center of the car platform, not a lidar origin, wheel axle,
  or chassis corner.
- Frame: `FIELD_GLOBAL`; car forward is `+X`, left is `+Y`, and yaw is
  counter-clockwise positive.
- Units: X/Y in cm, yaw in 0.1 degrees, velocity in cm/s if valid.
- Publish `CAR_POSE` only when position, calibration, and yaw validity bits
  are truthful. Use one nonzero `CalibrationId` for a task and never change it
  during that task.
- Pi-to-MCU local pose ingress uses UART4 (`PC11=MCU RX`) at 115200 8N1. A
  native `FIELD_GLOBAL`, position-valid, yaw-valid, calibrated pose with a
  nonzero `CalibrationId` is eligible for task coordination. Legacy
  `FA...AB` pose data remains display-only. Treat a 250 ms pose age as stale.
  Never make this input a local motor-start prerequisite: absent Pi/radar data
  defers `0x81` and coordinate assistance, but local gray/JY901 following
  remains available.

## 3. LoRa V2.3 Contract

Follow `F:\keil5\stm\ZET6 HAl\lidar_car\docs\D题_通用通信与接口规范_v2.3.docx`
exactly. Online frame Version remains `0x02`; V2.3 is the authoritative
document revision.

| Item | Car requirement |
| --- | --- |
| `CAR_POSE` type 0x80 | Broadcast at 10 Hz only while Pi pose is fresh; it is the shared wireless time base |
| `CAR_TASK_REQUEST` type 0x81 | Physical PG13 selects task 01 drop and PG9 selects task 02 dynamic landing. The local run may start without Pi; queue the same mission ID/sequence at most three times once a valid `CalibrationId` is present |
| `MISSION_STATUS` type 0x82 | Accept only the matching TaskType/MissionId. It is the authoritative one-shot stage transition |
| `FLIGHT_TELEMETRY` type 0x02 | Use ModeCode as continuous stage and lost-0x82 recovery only after this task's `0x81` ACK; it never starts or stops the car |
| Calibration type 0x83 | Accept only while car is not running; forward to Pi and ACK ground station only after Pi success ACK |
| Physical reset 0x85 | PG12 held for 2 s only after 12 s stopped broadcasts three identical `0x85` frames directly to ground. PG9 is not a maintenance key. |
| `MISSION_ABORT` type 0x84 | A second PG13 or PG9 press during an active run immediately turns motors off and sends three urgent, ACK-required abort frames |
| Car slot | 0-25 ms of each 100 ms cycle: pose, task retry, or required maintenance ACK |
| Air slot | 30-55 ms reserved for air-side response; car remains silent |
| Guard slot | 55-100 ms silent; no debug tunnel or free-running packet |

Deduplicate task requests and replies. Do not make the car stop, restart, or
perform flight decisions because an ACK arrives. Keep protocol logs compact:
`time_ms,type,seq,mission_id,crc_ok,age_ms,slot,event`.

## 4. MaixCam V2.3 Boundary

The flight controller accepts the car task, creates `ModeSeq`, and establishes
the MaixCam `CAMERA_MODE` session. The car preserves its `TaskType/MissionId`
in the V2.3 request and continues publishing pose, but never creates or
interprets `ModeSeq`, CAMERA_TARGET, CAMERA_ACTION, or CAMERA_ACTION_RESULT.
Camera ACK/timeout/target loss/result are air-side events and cannot stop or
restart the car line-following state machine.

## 5. Motion Policy

1. PG13 starts task one and PG9 starts task two only from the full-black A
   marker with fresh JY901. The same task keys are active-run safety stops.
   Do not use PG10 for task control.
2. Use gray line tracking as the primary path error. The current field image
   uses encoder LADRC inner loops for wheel speed and bounded direct JY901 yaw
   plus derived yaw-rate correction for heading. Pi/radar coordinates are
   auxiliary only: capture A when the calibrated pose is present at launch,
   use B/A for progress and stop preparation, and never feed coordinates into
   lateral control or independently stop the vehicle.
3. Measure the A-to-B course repeatedly and choose a target with margin below
   the required 15 s after the car begins moving. The target may be slower than
   later straight segments only if it still meets that deadline.
4. Start both tasks at approximately 120 mm/s. Task one may unlock the
   approximately 180 mm/s envelope only after B progress and the matched air
   task has passed `DROP_ACTION` then entered `RETURN_HOME`; task two unlocks
   at matched `INTERCEPT` or a later non-ABORT stage. Stale flight state or
   `ABORT` returns to the cooperative envelope, never stops the car.
5. Continue the full line loop while the air mission runs. The two task modes
   are selected before start, not by a ground-station control during the run.

## 6. State Outline

```text
IDLE
  -> BUTTON_TASK_SELECTED_AT_A + FRESH_JY901
  -> LINE_FOLLOW_A_TO_B + OPTIONAL_CAR_POSE_10HZ + DELAYED_TASK_REQUEST
  -> LINE_FOLLOW_REMAINING_LOOP + STATUS/POSE
  -> COMPLETE_OR_SAFE_STOP
```

Gate `BUTTON_TASK_SELECTED_AT_A` on a stable physical-button event, full-black
A marker, and fresh JY901. Gate `LINE_FOLLOW_A_TO_B` on valid gray input,
motor safety, and a selected speed profile. A valid Pi pose may add the
`CalibrationId` task request and coordinate assistance later; flight status
only adjusts the bounded speed profile and never owns the motor state machine.

## 7. Acceptance Checklist

- Gray mux address scan, all-white mask, decoded X1-X8 order, and map
  white/black polarity are measured on the competition tape.
- A-to-B repeated measurements satisfy the 15 s requirement with margin.
- Gray following remains stable at each chosen segment speed.
- When Pi pose is available, `CAR_POSE` is 10 Hz, CRC-valid, calibrated, and
  references platform center. Its absence must still allow a local button start.
- Task button produces exactly one local task selection and, once calibrated
  pose is available, exactly one task-request identity with no duplicate car
  motor action after retries or ACKs.
- LoRa slots and 5 ms protection are measured with the actual radio settings.
- Ground station shows position/status but cannot start, stop, or retune the
  car during the task.
- Flight-controller/MaixCam session evidence uses the same task type and
  mission ID as the car request; no car-side camera command exists.
