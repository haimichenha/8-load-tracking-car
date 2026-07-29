from __future__ import annotations

import argparse
import csv
import datetime as dt
import sys
from pathlib import Path

FIELDNAMES = [
    "date",
    "run_id",
    "stage",
    "param_group",
    "param_name",
    "old_value",
    "new_value",
    "test_condition",
    "result",
    "decision",
    "notes",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Append one tuning record row for STM32F103ZET6 staged tests.")
    parser.add_argument("csvfile", type=Path, help="CSV file to append tuning records")
    parser.add_argument("--run-id", required=True, help="Test run identifier")
    parser.add_argument("--stage", required=True, help="Stage name, e.g. C2/C3/C4/C5")
    parser.add_argument("--param-group", required=True, help="Parameter group, e.g. speed_loop / motion_test")
    parser.add_argument("--param-name", required=True, help="Parameter name")
    parser.add_argument("--old", required=True, help="Old value")
    parser.add_argument("--new", required=True, help="New value")
    parser.add_argument("--test-condition", required=True, help="Test condition summary")
    parser.add_argument("--result", required=True, help="Observed result summary")
    parser.add_argument("--decision", required=True, choices=["keep", "rollback", "retest", "next-stage"], help="Decision after the run")
    parser.add_argument("--notes", default="", help="Extra notes")
    return parser.parse_args()


def ensure_header(path: Path) -> None:
    if path.exists() and path.stat().st_size > 0:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=FIELDNAMES)
        writer.writeheader()


def main() -> int:
    args = parse_args()
    ensure_header(args.csvfile)

    row = {
        "date": dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "run_id": args.run_id,
        "stage": args.stage,
        "param_group": args.param_group,
        "param_name": args.param_name,
        "old_value": args.old,
        "new_value": args.new,
        "test_condition": args.test_condition,
        "result": args.result,
        "decision": args.decision,
        "notes": args.notes,
    }

    with args.csvfile.open("a", encoding="utf-8-sig", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=FIELDNAMES)
        writer.writerow(row)

    print("record_appended")
    print(",".join(str(row[name]) for name in FIELDNAMES))
    return 0


if __name__ == "__main__":
    sys.exit(main())
