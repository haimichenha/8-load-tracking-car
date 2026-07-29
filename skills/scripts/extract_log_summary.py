from __future__ import annotations

import argparse
import csv
import io
import re
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable

NUMERIC_FIELDS = {
    "test_pwm", "enc_delta", "speed", "pwm_l", "pwm_r", "enc_l_delta", "enc_r_delta",
    "speed_l", "speed_r", "duration_ms", "target_l", "target_r", "err_l", "err_r",
    "i_l", "i_r", "yaw_start", "yaw_now"
}

MOTION_C35_HEADER = [
    "time_ms", "run_id", "base_pwm", "trim", "pwm_l", "pwm_r", "duration_ms",
    "enc_l_delta", "enc_r_delta", "speed_l", "speed_r", "event"
]

ENCODER_C36_HEADER = [
    "time_ms", "run_id", "pwm", "mode", "pwm_l", "pwm_r", "duration_ms",
    "enc_l_delta", "enc_r_delta", "speed_l", "speed_r", "left_ab", "right_ab", "event"
]

SPEED_C4_HEADER = [
    "time_ms", "run_id", "mode", "target_l", "target_r", "speed_l", "speed_r",
    "err_l", "err_r", "pwm_l", "pwm_r", "i_l", "i_r", "sat_l", "sat_r",
    "left_ab", "right_ab", "event"
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize STM32F103ZET6 staged CSV logs for C2/C3/C4/C5 tests.")
    parser.add_argument("logfile", type=Path, help="Path to exported CSV log")
    parser.add_argument("--stage", choices=["C2", "C3", "C3.5", "C3.6", "C4", "C5"], help="Filter one stage only")
    return parser.parse_args()


def as_float(value: str) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def rows_for_stage(rows: Iterable[dict[str, str]], stage: str | None) -> list[dict[str, str]]:
    if stage is None:
        return list(rows)
    return [row for row in rows if row.get("stage", "").strip() == stage]


def load_rows(logfile: Path) -> list[dict[str, str]]:
    text = logfile.read_text(encoding="utf-8", errors="ignore")
    speed_rows: list[dict[str, str]] = []
    for match in re.finditer(r"speed_c4\.log,([^\r\n]+)", text):
        fields = [item.strip() for item in match.group(1).split(",")]
        if len(fields) != len(SPEED_C4_HEADER):
            continue
        row = dict(zip(SPEED_C4_HEADER, fields))
        row["stage"] = "C4"
        speed_rows.append(row)
    if speed_rows:
        return speed_rows

    encoder_rows: list[dict[str, str]] = []
    for match in re.finditer(r"encoder_c36\.log,([^\r\n]+)", text):
        fields = [item.strip() for item in match.group(1).split(",")]
        if len(fields) != len(ENCODER_C36_HEADER):
            continue
        row = dict(zip(ENCODER_C36_HEADER, fields))
        row["stage"] = "C3.6"
        encoder_rows.append(row)
    if encoder_rows:
        return encoder_rows

    motion_rows: list[dict[str, str]] = []
    for match in re.finditer(r"motion_c35\.log,([^\r\n]+)", text):
        fields = [item.strip() for item in match.group(1).split(",")]
        if len(fields) != len(MOTION_C35_HEADER):
            continue
        row = dict(zip(MOTION_C35_HEADER, fields))
        row["stage"] = "C3.5"
        motion_rows.append(row)
    if motion_rows:
        return motion_rows

    reader = csv.DictReader(io.StringIO(text))
    return list(reader)


def summarize_c2(rows: list[dict[str, str]]) -> None:
    wheels: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        wheels[row.get("wheel", "?")].append(row)

    print("summary_stage=C2")
    for wheel, items in sorted(wheels.items()):
        moved_pwms = sorted({int(float(row["test_pwm"])) for row in items if row.get("moved", "0") in {"1", "true", "True", "yes", "YES"}})
        stable_pwm = moved_pwms[0] if moved_pwms else "NA"
        speeds = [as_float(row.get("speed", "")) for row in items]
        speeds = [value for value in speeds if value is not None]
        print(f"wheel={wheel} rows={len(items)} stable_min_pwm={stable_pwm} avg_speed={(statistics.mean(speeds) if speeds else 0):.3f}")


def summarize_c3(rows: list[dict[str, str]]) -> None:
    buckets: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        key = (row.get("pwm_l", "?"), row.get("pwm_r", "?"))
        buckets[key].append(row)

    print("summary_stage=C3")
    for (pwm_l, pwm_r), items in sorted(buckets.items(), key=lambda item: (float(item[0][0]), float(item[0][1]))):
        speed_l = [as_float(row.get("speed_l", "")) for row in items]
        speed_r = [as_float(row.get("speed_r", "")) for row in items]
        speed_l = [value for value in speed_l if value is not None]
        speed_r = [value for value in speed_r if value is not None]
        avg_l = statistics.mean(speed_l) if speed_l else 0.0
        avg_r = statistics.mean(speed_r) if speed_r else 0.0
        diff = avg_l - avg_r
        print(f"pwm_l={pwm_l} pwm_r={pwm_r} rows={len(items)} avg_speed_l={avg_l:.3f} avg_speed_r={avg_r:.3f} diff={diff:.3f}")


def summarize_c35(rows: list[dict[str, str]]) -> None:
    buckets: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if row.get("event") in {"START", "STOP"}:
            buckets[row.get("trim", "?")].append(row)

    print("summary_stage=C3.5")
    for trim, items in sorted(buckets.items(), key=lambda item: float(item[0])):
        starts = [row for row in items if row.get("event") == "START"]
        stops = [row for row in items if row.get("event") == "STOP"]
        enc_l = [as_float(row.get("enc_l_delta", "")) for row in stops]
        enc_r = [as_float(row.get("enc_r_delta", "")) for row in stops]
        enc_l = [value for value in enc_l if value is not None]
        enc_r = [value for value in enc_r if value is not None]
        avg_l = statistics.mean(enc_l) if enc_l else 0.0
        avg_r = statistics.mean(enc_r) if enc_r else 0.0
        print(f"trim={trim} starts={len(starts)} stops={len(stops)} avg_enc_l={avg_l:.3f} avg_enc_r={avg_r:.3f} diff={avg_l - avg_r:.3f}")


def summarize_c36(rows: list[dict[str, str]]) -> None:
    buckets: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if row.get("event") in {"START", "STOP", "RUN"}:
            buckets[row.get("mode", "?")].append(row)

    print("summary_stage=C3.6")
    for mode in ["L", "R", "B", "?"]:
        items = buckets.get(mode, [])
        if not items:
            continue
        starts = [row for row in items if row.get("event") == "START"]
        stops = [row for row in items if row.get("event") == "STOP"]
        enc_l = [as_float(row.get("enc_l_delta", "")) for row in stops]
        enc_r = [as_float(row.get("enc_r_delta", "")) for row in stops]
        enc_l = [value for value in enc_l if value is not None]
        enc_r = [value for value in enc_r if value is not None]
        avg_l = statistics.mean(enc_l) if enc_l else 0.0
        avg_r = statistics.mean(enc_r) if enc_r else 0.0
        ab_l = Counter(row.get("left_ab", "?") for row in items)
        ab_r = Counter(row.get("right_ab", "?") for row in items)
        print(
            f"mode={mode} starts={len(starts)} stops={len(stops)} "
            f"avg_enc_l={avg_l:.3f} avg_enc_r={avg_r:.3f} "
            f"left_ab={dict(sorted(ab_l.items()))} right_ab={dict(sorted(ab_r.items()))}"
        )

    print("diagnosis_hint=")
    print("- mode=L 左轮转时 enc_l_delta 应明显变化，mode=R 右轮转时 enc_r_delta 应明显变化")
    print("- 某轮转但对应 delta=0：先查编码器供电、A/B 接线、GPIO 输入配置")
    print("- 某轮转但另一侧 delta 变化：左右编码器线可能接反")
    print("- delta 有变化但符号反：后续只改符号映射，不要调 PID")


def summarize_c4(rows: list[dict[str, str]]) -> None:
    print("summary_stage=C4")
    buckets: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if row.get("event") in {"RUN", "STOP"}:
            buckets[row.get("mode", "?")].append(row)

    for mode in ["L", "R", "B", "?"]:
        items = buckets.get(mode, [])
        if not items:
            continue
        err_l = [abs(as_float(row.get("err_l", "")) or 0.0) for row in items if row.get("event") == "RUN"]
        err_r = [abs(as_float(row.get("err_r", "")) or 0.0) for row in items if row.get("event") == "RUN"]
        speed_l = [as_float(row.get("speed_l", "")) for row in items if row.get("event") == "RUN"]
        speed_r = [as_float(row.get("speed_r", "")) for row in items if row.get("event") == "RUN"]
        pwm_l = [as_float(row.get("pwm_l", "")) for row in items if row.get("event") == "RUN"]
        pwm_r = [as_float(row.get("pwm_r", "")) for row in items if row.get("event") == "RUN"]
        speed_l = [value for value in speed_l if value is not None]
        speed_r = [value for value in speed_r if value is not None]
        pwm_l = [value for value in pwm_l if value is not None]
        pwm_r = [value for value in pwm_r if value is not None]
        sat_events = sum(1 for row in items if row.get("sat_l", "0") not in {"0", "false", "False", ""} or row.get("sat_r", "0") not in {"0", "false", "False", ""})
        print(
            f"mode={mode} rows={len(items)} "
            f"mean_abs_err_l={(statistics.mean(err_l) if err_l else 0):.3f} "
            f"mean_abs_err_r={(statistics.mean(err_r) if err_r else 0):.3f} "
            f"avg_speed_l={(statistics.mean(speed_l) if speed_l else 0):.3f} "
            f"avg_speed_r={(statistics.mean(speed_r) if speed_r else 0):.3f} "
            f"avg_pwm_l={(statistics.mean(pwm_l) if pwm_l else 0):.3f} "
            f"avg_pwm_r={(statistics.mean(pwm_r) if pwm_r else 0):.3f} "
            f"sat_events={sat_events}"
        )


def summarize_c5(rows: list[dict[str, str]]) -> None:
    modes = Counter(row.get("turn_mode", "?") for row in rows)
    print("summary_stage=C5")
    for mode, count in sorted(modes.items()):
        yaw_changes = []
        for row in rows:
            if row.get("turn_mode", "?") != mode:
                continue
            start = as_float(row.get("yaw_start", ""))
            now = as_float(row.get("yaw_now", ""))
            if start is not None and now is not None:
                yaw_changes.append(now - start)
        avg_yaw = statistics.mean(yaw_changes) if yaw_changes else 0.0
        print(f"turn_mode={mode} rows={count} avg_yaw_delta={avg_yaw:.3f}")


def main() -> int:
    args = parse_args()
    if not args.logfile.exists():
        print(f"log file not found: {args.logfile}", file=sys.stderr)
        return 1

    rows = load_rows(args.logfile)
    if args.stage and args.stage not in {"C3.5", "C3.6"}:
        rows = rows_for_stage(rows, args.stage)

    if not rows:
        print("no rows matched")
        return 1

    stage = args.stage or rows[0].get("stage", "")
    if stage == "C2":
        summarize_c2(rows)
    elif stage == "C3":
        summarize_c3(rows)
    elif stage == "C3.5":
        summarize_c35(rows)
    elif stage == "C3.6":
        summarize_c36(rows)
    elif stage == "C4":
        summarize_c4(rows)
    elif stage == "C5":
        summarize_c5(rows)
    else:
        counts = Counter(row.get("stage", "UNKNOWN") for row in rows)
        print("summary_all_stages")
        for key, value in sorted(counts.items()):
            print(f"stage={key} rows={value}")

    events = Counter(row.get("event", "") for row in rows if row.get("event"))
    if events:
        print("events=")
        for name, count in sorted(events.items()):
            print(f"- {name}: {count}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
