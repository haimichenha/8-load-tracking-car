from __future__ import annotations

from collections import defaultdict
from pathlib import Path


def main() -> None:
    p = Path("more.log")
    text = p.read_text(errors="ignore")

    header = None
    for ln in text.splitlines():
        if "seq,test_id,time_ms" in ln:
            header = ln.strip().split(",")
            break

    if not header:
        header = [
            "seq",
            "test_id",
            "time_ms",
            "error",
            "sensor",
            "active",
            "cnt",
            "pwmL",
            "pwmR",
            "speedL",
            "speedR",
            "tgtL",
            "tgtR",
            "flags",
            "etSlow",
            "sat",
            "kpScale10",
            "dtMs",
            "errSpdL10",
            "errSpdR10",
            "turnOut",
            "dynBase",
        ]

    idx = {k: i for i, k in enumerate(header)}

    rows: list[list[str]] = []
    for ln in text.splitlines():
        ln = ln.strip()
        if not ln or not ln[0].isdigit() or "," not in ln:
            continue
        parts = ln.split(",")
        if len(parts) != len(header):
            continue
        rows.append(parts)

    print(f"rows={len(rows)} cols={len(header)}")

    st = defaultdict(
        lambda: {
            "n": 0,
            "tmax": 0,
            "lost": 0,
            "turnSat": 0,
            "maxLostMs": 0,
            "maxLostStart": None,
            "maxLostEnd": None,
            "_inLost": False,
            "_lostStart": 0,
        }
    )

    for r in rows:
        tid = int(r[idx["test_id"]])
        t = int(r[idx["time_ms"]])
        cnt = int(r[idx["cnt"]])
        sat_raw = r[idx["sat"]]
        sat = int(sat_raw, 0) if isinstance(sat_raw, str) else int(sat_raw)

        s = st[tid]
        s["n"] += 1
        if t > s["tmax"]:
            s["tmax"] = t

        lost = cnt == 0
        if lost:
            s["lost"] += 1

        if (sat & 1) != 0:
            s["turnSat"] += 1

        if lost and not s["_inLost"]:
            s["_inLost"] = True
            s["_lostStart"] = t

        if (not lost) and s["_inLost"]:
            s["_inLost"] = False
            dur = t - s["_lostStart"]
            if dur > s["maxLostMs"]:
                s["maxLostMs"] = dur
                s["maxLostStart"] = s["_lostStart"]
                s["maxLostEnd"] = t

    for tid, s in st.items():
        if s["_inLost"]:
            dur = s["tmax"] - s["_lostStart"]
            if dur > s["maxLostMs"]:
                s["maxLostMs"] = dur
                s["maxLostStart"] = s["_lostStart"]
                s["maxLostEnd"] = s["tmax"]

    print("test  N     dur(ms)  lost%   turnSat%  maxLost(ms)  start-end")
    for tid in sorted(st):
        s = st[tid]
        lostpct = 100.0 * s["lost"] / s["n"] if s["n"] else 0.0
        satpct = 100.0 * s["turnSat"] / s["n"] if s["n"] else 0.0
        start = s["maxLostStart"]
        end = s["maxLostEnd"]
        se = f"{start}-{end}" if start is not None else "-"
        print(
            f"T{tid:<3} {s['n']:<5} {s['tmax']:<7} {lostpct:6.2f} {satpct:9.2f} {s['maxLostMs']:10}  {se}"
        )


if __name__ == "__main__":
    main()
