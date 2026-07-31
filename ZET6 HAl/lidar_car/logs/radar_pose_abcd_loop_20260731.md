# Radar Pose A/B/C/D Loop Record - 2026-07-31

## Acquisition context

- Pi stack: `phase8_slam_rf2o_bridge_serial.launch.py`, restarted while the
  vehicle was stationary at A.
- Coordinate frame: the pose bridge locked its local origin at A from raw
  `(0.09 cm, 0.12 cm, 0.12 deg)` and emitted the first valid local pose as
  `(0.0 cm, 0.0 cm, 0.0 deg)`.
- Source log: `/home/ubuntu/lidar_logs/phase8_slam_bridge_serial_20260731_161110.log`.
- Units: X/Y are centimetres; yaw is degrees in `[-180, 180)`.
- Only stationary `bridge_out valid=1` samples were used below. Reported point
  values are the centre of each captured stable window.

## Point coordinates

| Point | X (cm) | Y (cm) | Yaw (deg) | Valid samples | Pose age (ms) | Window stability |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| A start / origin | 0.0 | 0.0 | 0.0 | first valid frame | 118 | Bridge origin locked at A |
| B | -151.3 | -8.1 | -0.7 | 12 | 19-174 | X 0.4 cm, Y 0.2 cm, yaw 0.1 deg |
| C | -165.7 | 150.7 | -178.1 | 12 | 19-169 | X 0.3 cm, Y 0.3 cm, yaw 0.1 deg |
| D | -16.5 | 164.2 | -179.4 | 12 | 19-171 | X 0.1 cm, Y 0.2 cm, yaw 0.1 deg |

`-178.1 deg` and `-179.4 deg` are equivalent to `181.9 deg` and `180.6 deg`
when expressed in a `[0, 360)` convention.

## Map relation

The endpoint geometry is consistent with the approximately 150 cm by 150 cm
course shown in `docs/地图描述.png`:

| Chord | Measured length (cm) |
| --- | ---: |
| A start -> B | 151.5 |
| B -> C | 159.5 |
| C -> D | 149.8 |
| D -> A start | 165.0 |

The corresponding waypoint turns at B, C, and D are approximately 88 deg,
90 deg, and 89 deg. B/C/D are retained as usable route-reference coordinates
for this origin frame.

## Heading convention for current integration

- A and B are the same straight-side heading and should be treated as approximately
  `0 deg` (the measured B value `-0.7 deg` is within the stationary tolerance).
- C and D are the opposite straight-side heading and should be treated as approximately
  `180 deg`; the measured `-178.1 deg` and `-179.4 deg` are the wrapped representation
  of that direction.
- These headings and coordinates agree with the road outline in `docs/地图描述.png` and
  are now the active car/air integration reference. They are not a replacement for the
  gray A marker when deciding the final stop.

## Return-A observation

The returning A measurement is recorded separately and does not overwrite the
start-origin coordinate:

| Item | Value |
| --- | ---: |
| Stable return-A centre | X = 5.8 cm, Y = 40.2 cm, yaw = -11.8 deg |
| Return samples | 18 valid samples, age 18-170 ms |
| Position closure relative to A start | approximately 40.6 cm |
| Yaw closure relative to A start | approximately -11.8 deg |

This records the observed return-A offset for later investigation while keeping
the A/B/C/D reference coordinates above unchanged.
