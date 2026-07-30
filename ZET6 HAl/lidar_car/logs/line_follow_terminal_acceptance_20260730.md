# Line-follow terminal acceptance — 2026-07-30

## Acceptance run

- Image: `LineFollowJY901Debug`.
- Frozen evidence: `line_follow_frozen_20260730-190434.log`.
- Run window: `t_ms=5995..35203` (`29.208 s`); `254` samples.
- Result: automatic stop accepted with `TERMINAL_REASON=A_RETURN_AFTER_2_ARCS`.
- JY901: `GYRO_STALE_RECORDS=0`; terminal status reported
  `lap_yaw_travel_tenths=-3323` and front/rear motors disabled.

## Terminal correction evidence

The preceding run stopped incorrectly in B--C with
`A_ARC_MARK_RETURN`. The official map contains two clockwise semicircles
(B--C and D--A), so a gray-only outer-right/wide signature cannot identify
A. The accepted image accumulates the direct JY901 yaw and enables the curved
A marker only after `330 deg` clockwise progress. B--C is therefore excluded;
the return A marker is accepted after the two arc phases.

## Control-quality observation for next iteration

- `|gray_err_x100| >= 400`: `182 / 254` samples (`71.7%`).
- Stable center `X4|X5=0x18`: `22 / 254` samples (`8.7%`).
- Maximum total differential: `2200 cps`; maximum target split: `2200 cps`.

The car completed the loop, but it spent too long using X7/X8-side recovery in
curves. Next work is **center attraction / anti-edge recovery** inside the
gray steering reference. It must not alter motor direction, speed-loop
polarity, JY901 source, or the accepted terminal gate.
