# D-Task Car Mission Contract

## 1. Car-Side Role

The car is responsible for physical start buttons, gray line following, local
motion safety, Pi pose intake, LoRa V2.2 transmission, calibration forwarding,
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
- Pi-to-MCU is a local 115200 8N1 V2.2 link at 10 Hz. The MCU must validate
  length, CRC, source/destination, sequence, freshness, and calibration before
  forwarding to LoRa.

## 3. LoRa V2.2 Contract

Follow `F:\keil5\stm\docs\D题_通用通信与接口规范_v2.2.docx` exactly.

| Item | Car requirement |
| --- | --- |
| `CAR_POSE` type 0x80 | Broadcast at 10 Hz; it is the shared wireless time base |
| `CAR_TASK_REQUEST` type 0x81 | Physical button selects task 01 drop or 02 dynamic landing; same mission ID/sequence retries at most three times |
| Calibration type 0x83 | Accept only while car is not running; forward to Pi and ACK ground station only after Pi success ACK |
| Car slot | 0-25 ms of each 100 ms cycle: pose, task retry, or required maintenance ACK |
| Air slot | 30-55 ms reserved for air-side response; car remains silent |
| Guard slot | 55-100 ms silent; no debug tunnel or free-running packet |

Deduplicate task requests and replies. Do not make the car stop, restart, or
perform flight decisions because an ACK arrives. Keep protocol logs compact:
`time_ms,type,seq,mission_id,crc_ok,age_ms,slot,event`.

## 4. MaixCam V2.2 Boundary

The flight controller accepts the car task, creates `ModeSeq`, and establishes
the MaixCam `CAMERA_MODE` session. The car preserves its `TaskType/MissionId`
in the V2.2 request and continues publishing pose, but never creates or
interprets `ModeSeq`, CAMERA_TARGET, CAMERA_ACTION, or CAMERA_ACTION_RESULT.
Camera ACK/timeout/target loss/result are air-side events and cannot stop or
restart the car line-following state machine.

## 5. Motion Policy

1. The physical button starts the selected task and the car motion sequence.
2. Use gray line tracking as the primary path error. Use encoder inner loops
   for wheel speed and bounded gyro correction for heading.
3. Measure the A-to-B course repeatedly and choose a target with margin below
   the required 15 s after the car begins moving. The target may be slower than
   later straight segments only if it still meets that deadline.
4. Give faster sections their own tested limits; do not copy the A-to-B value
   blindly. Curves, line-loss recovery, payload motion, and platform stability
   constrain their speed.
5. Continue the full line loop while the air mission runs. The two task modes
   are selected before start, not by a ground-station control during the run.

## 6. State Outline

```text
IDLE
  -> PREFLIGHT_CALIBRATED
  -> BUTTON_TASK_SELECTED
  -> LINE_FOLLOW_A_TO_B + CAR_POSE_10HZ + TASK_REQUEST_RETRY
  -> LINE_FOLLOW_REMAINING_LOOP + STATUS/POSE
  -> COMPLETE_OR_SAFE_STOP
```

Gate `BUTTON_TASK_SELECTED` on a stable physical-button event and a valid
local pose/calibration policy. Gate `LINE_FOLLOW_A_TO_B` on valid gray input,
motor safety, and a selected speed profile. Flight status is displayed and
logged but does not own the motor state machine.

## 7. Acceptance Checklist

- Gray mux address scan, all-white mask, decoded X1-X8 order, and map
  white/black polarity are measured on the competition tape.
- A-to-B repeated measurements satisfy the 15 s requirement with margin.
- Gray following remains stable at each chosen segment speed.
- `CAR_POSE` is 10 Hz, CRC-valid, calibrated, and references platform center.
- Task button produces exactly one task request identity and no duplicate car
  motor action after retries or ACKs.
- LoRa slots and 5 ms protection are measured with the actual radio settings.
- Ground station shows position/status but cannot start, stop, or retune the
  car during the task.
- Flight-controller/MaixCam session evidence uses the same task type and
  mission ID as the car request; no car-side camera command exists.
