#!/usr/bin/env python3
"""Parse more.log with line-break handling, extract intersection and sharp turn events."""

import re
import sys

LOG_FILE = r"F:\keil5\stm\more.log"

# Read the file
with open(LOG_FILE, 'r', encoding='utf-8', errors='replace') as f:
    raw_lines = f.readlines()

# Step 1: Remove timestamp lines and join broken lines
cleaned = []
for line in raw_lines:
    line = line.rstrip('\r\n')
    # Skip empty lines
    if not line.strip():
        continue
    # Skip timestamp lines like [2026-02-13 09:35:40.010]# RECV ASCII>
    if re.match(r'\[2026-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}\.\d+\]', line):
        continue
    # Skip header/marker lines
    if line.startswith('#'):
        cleaned.append(line)
        continue
    cleaned.append(line)

# Step 2: Join broken CSV lines
# A valid data line starts with a number (seq field)
# A broken continuation doesn't start with a number at the beginning
joined = []
for line in cleaned:
    if line.startswith('#'):
        joined.append(line)
        continue
    # Check if this line starts with a number (new data line)
    if re.match(r'^\d+,', line):
        joined.append(line)
    else:
        # This is a continuation of the previous line
        if joined and not joined[-1].startswith('#'):
            joined[-1] += line
        else:
            # Orphan fragment, try to append anyway
            if joined:
                joined[-1] += line

# Step 3: Parse CSV data lines
FIELDS = ['seq','test_id','time_ms','error','sensor','active','cnt','pwmL','pwmR',
          'speedL','speedR','tgtL','tgtR','flags','etSlow','sat','kpScale10','dtMs',
          'errSpdL10','errSpdR10','turnOut','dynBase']

data_rows = []
test_markers = []

for line in joined:
    if line.startswith('#TEST,'):
        m = re.match(r'#TEST,(\d+)', line)
        if m:
            test_markers.append(int(m.group(1)))
        continue
    if line.startswith('#'):
        continue

    parts = line.split(',')
    if len(parts) >= 22:
        try:
            row = {}
            row['seq'] = int(parts[0])
            row['test_id'] = int(parts[1])
            row['time_ms'] = int(parts[2])
            row['error'] = int(parts[3])
            row['sensor'] = parts[4].strip()
            row['active'] = parts[5].strip()
            row['cnt'] = int(parts[6])
            row['pwmL'] = int(parts[7])
            row['pwmR'] = int(parts[8])
            row['speedL'] = int(parts[9])
            row['speedR'] = int(parts[10])
            row['tgtL'] = int(parts[11])
            row['tgtR'] = int(parts[12])
            row['flags'] = parts[13].strip()
            row['etSlow'] = int(parts[14])
            row['sat'] = parts[15].strip()
            row['kpScale10'] = int(parts[16])
            row['dtMs'] = int(parts[17])
            row['errSpdL10'] = int(parts[18])
            row['errSpdR10'] = int(parts[19])
            row['turnOut'] = int(parts[20])
            row['dynBase'] = int(parts[21])
            data_rows.append(row)
        except (ValueError, IndexError) as e:
            # Skip malformed lines
            pass

print(f"Total parsed data rows: {len(data_rows)}")
print(f"Test markers: {test_markers}")
print(f"Seq range: {data_rows[0]['seq']} - {data_rows[-1]['seq']}")
print(f"Test IDs: {sorted(set(r['test_id'] for r in data_rows))}")
print()

# Step 4: Find all frames with flags containing 0x40 (intersection)
print("=" * 120)
print("PROBLEM 1: INTERSECTION DETECTION (flags & 0x40)")
print("=" * 120)

intersection_indices = []
for idx, row in enumerate(data_rows):
    flags_val = int(row['flags'], 16)
    if flags_val & 0x40:
        intersection_indices.append(idx)

print(f"\nFound {len(intersection_indices)} frames with 0x40 flag")
print(f"Intersection frame seqs: {[data_rows[i]['seq'] for i in intersection_indices]}")

# Group consecutive intersection frames
groups = []
if intersection_indices:
    current_group = [intersection_indices[0]]
    for i in range(1, len(intersection_indices)):
        if intersection_indices[i] - intersection_indices[i-1] <= 5:
            current_group.append(intersection_indices[i])
        else:
            groups.append(current_group)
            current_group = [intersection_indices[i]]
    groups.append(current_group)

print(f"Intersection groups: {len(groups)}")

for gidx, group in enumerate(groups):
    start_idx = max(0, group[0] - 20)
    end_idx = min(len(data_rows) - 1, group[-1] + 20)

    print(f"\n{'='*120}")
    print(f"INTERSECTION GROUP {gidx+1}: frames seq {data_rows[group[0]]['seq']}-{data_rows[group[-1]]['seq']}")
    print(f"Context: seq {data_rows[start_idx]['seq']} to {data_rows[end_idx]['seq']}")
    print(f"{'='*120}")

    # Print header
    print(f"{'seq':>5} {'time':>6} {'err':>4} {'sensor':>6} {'active':>6} {'cnt':>3} {'flags':>6} {'turnOut':>8} {'dynBase':>8} {'tgtL':>5} {'tgtR':>5} {'pwmL':>5} {'pwmR':>5} {'spdL':>5} {'spdR':>5} {'etSlow':>6} {'sat':>6} {'kpS10':>6} {'dt':>3} {'MARK':>8}")
    print("-" * 120)

    for i in range(start_idx, end_idx + 1):
        r = data_rows[i]
        flags_val = int(r['flags'], 16)
        mark = ""
        if flags_val & 0x40: mark += "CROSS "
        if flags_val & 0x08: mark += "LOST "
        if flags_val & 0x10: mark += "SLOW "
        if flags_val & 0x80: mark += "RECOV "
        if flags_val & 0x20: mark += "GAP "
        if i in intersection_indices: mark = ">>>" + mark

        print(f"{r['seq']:>5} {r['time_ms']:>6} {r['error']:>4} {r['sensor']:>6} {r['active']:>6} {r['cnt']:>3} {r['flags']:>6} {r['turnOut']:>8} {r['dynBase']:>8} {r['tgtL']:>5} {r['tgtR']:>5} {r['pwmL']:>5} {r['pwmR']:>5} {r['speedL']:>5} {r['speedR']:>5} {r['etSlow']:>6} {r['sat']:>6} {r['kpScale10']:>6} {r['dtMs']:>3} {mark:>8}")

# Step 5: Find lost-line segments (flags & 0x08 or cnt==0)
print("\n\n" + "=" * 120)
print("PROBLEM 2: SHARP TURN / LOST LINE ANALYSIS")
print("=" * 120)

# Find all lost-line segments
lost_segments = []
in_lost = False
seg_start = None
for idx, row in enumerate(data_rows):
    flags_val = int(row['flags'], 16)
    is_lost = (flags_val & 0x08) != 0
    if is_lost and not in_lost:
        seg_start = idx
        in_lost = True
    elif not is_lost and in_lost:
        lost_segments.append((seg_start, idx - 1))
        in_lost = False
if in_lost:
    lost_segments.append((seg_start, len(data_rows) - 1))

print(f"\nFound {len(lost_segments)} lost-line segments")

for sidx, (seg_s, seg_e) in enumerate(lost_segments):
    seg_len = seg_e - seg_s + 1
    # Show context: 15 frames before, the segment, 30 frames after
    ctx_start = max(0, seg_s - 15)
    ctx_end = min(len(data_rows) - 1, seg_e + 30)

    print(f"\n{'='*120}")
    print(f"LOST SEGMENT {sidx+1}: frames seq {data_rows[seg_s]['seq']}-{data_rows[seg_e]['seq']} ({seg_len} frames)")
    print(f"  Before lost: error={data_rows[max(0,seg_s-1)]['error']}")
    if seg_e + 1 < len(data_rows):
        print(f"  After lost: error={data_rows[seg_e+1]['error']}")
    print(f"Context: seq {data_rows[ctx_start]['seq']} to {data_rows[ctx_end]['seq']}")
    print(f"{'='*120}")

    print(f"{'seq':>5} {'time':>6} {'err':>4} {'sensor':>6} {'active':>6} {'cnt':>3} {'flags':>6} {'turnOut':>8} {'dynBase':>8} {'tgtL':>5} {'tgtR':>5} {'pwmL':>5} {'pwmR':>5} {'spdL':>5} {'spdR':>5} {'etSlow':>6} {'kpS10':>6} {'dt':>3} {'MARK':>12}")
    print("-" * 120)

    for i in range(ctx_start, ctx_end + 1):
        r = data_rows[i]
        flags_val = int(r['flags'], 16)
        mark = ""
        if flags_val & 0x40: mark += "CROSS "
        if flags_val & 0x08: mark += "LOST "
        if flags_val & 0x10: mark += "SLOW "
        if flags_val & 0x80: mark += "RECOV "
        if flags_val & 0x20: mark += "GAP "
        if i >= seg_s and i <= seg_e: mark = ">>>" + mark

        print(f"{r['seq']:>5} {r['time_ms']:>6} {r['error']:>4} {r['sensor']:>6} {r['active']:>6} {r['cnt']:>3} {r['flags']:>6} {r['turnOut']:>8} {r['dynBase']:>8} {r['tgtL']:>5} {r['tgtR']:>5} {r['pwmL']:>5} {r['pwmR']:>5} {r['speedL']:>5} {r['speedR']:>5} {r['etSlow']:>6} {r['kpScale10']:>6} {r['dtMs']:>3} {mark:>12}")

# Step 6: Check for "turn back" pattern - lost line, recover, then lost again quickly
print("\n\n" + "=" * 120)
print("DOUBLE-LOST ANALYSIS (potential turnaround)")
print("=" * 120)

for sidx in range(len(lost_segments) - 1):
    seg1_end = lost_segments[sidx][1]
    seg2_start = lost_segments[sidx + 1][0]
    gap_frames = seg2_start - seg1_end - 1

    if gap_frames <= 30:  # Within 30 frames (300ms) of each other
        seg1_s, seg1_e = lost_segments[sidx]
        seg2_s, seg2_e = lost_segments[sidx + 1]

        # Check error direction before each lost segment
        err_before_1 = data_rows[max(0, seg1_s - 1)]['error'] if seg1_s > 0 else 0
        err_after_1 = data_rows[min(len(data_rows)-1, seg1_e + 1)]['error'] if seg1_e + 1 < len(data_rows) else 0
        err_before_2 = data_rows[max(0, seg2_s - 1)]['error'] if seg2_s > 0 else 0

        print(f"\nPOTENTIAL TURNAROUND between segments {sidx+1} and {sidx+2}:")
        print(f"  Seg{sidx+1}: seq {data_rows[seg1_s]['seq']}-{data_rows[seg1_e]['seq']} ({seg1_e-seg1_s+1} frames)")
        print(f"  Gap: {gap_frames} frames")
        print(f"  Seg{sidx+2}: seq {data_rows[seg2_s]['seq']}-{data_rows[seg2_e]['seq']} ({seg2_e-seg2_s+1} frames)")
        print(f"  Error before seg1: {err_before_1}, after seg1: {err_after_1}, before seg2: {err_before_2}")

        # Show the full context
        ctx_start = max(0, seg1_s - 10)
        ctx_end = min(len(data_rows) - 1, seg2_e + 15)

        print(f"\n{'seq':>5} {'time':>6} {'err':>4} {'sensor':>6} {'active':>6} {'cnt':>3} {'flags':>6} {'turnOut':>8} {'dynBase':>8} {'tgtL':>5} {'tgtR':>5} {'pwmL':>5} {'pwmR':>5} {'spdL':>5} {'spdR':>5} {'MARK':>15}")
        print("-" * 120)

        for i in range(ctx_start, ctx_end + 1):
            r = data_rows[i]
            flags_val = int(r['flags'], 16)
            mark = ""
            if flags_val & 0x40: mark += "CROSS "
            if flags_val & 0x08: mark += "LOST "
            if flags_val & 0x10: mark += "SLOW "
            if flags_val & 0x80: mark += "RECOV "
            if flags_val & 0x20: mark += "GAP "
            if (i >= seg1_s and i <= seg1_e) or (i >= seg2_s and i <= seg2_e):
                mark = ">>>" + mark

            print(f"{r['seq']:>5} {r['time_ms']:>6} {r['error']:>4} {r['sensor']:>6} {r['active']:>6} {r['cnt']:>3} {r['flags']:>6} {r['turnOut']:>8} {r['dynBase']:>8} {r['tgtL']:>5} {r['tgtR']:>5} {r['pwmL']:>5} {r['pwmR']:>5} {r['speedL']:>5} {r['speedR']:>5} {mark:>15}")

# Step 7: Track error direction changes (potential turnaround evidence)
print("\n\n" + "=" * 120)
print("ERROR SIGN REVERSAL ANALYSIS")
print("=" * 120)

# Find frames where error changes sign significantly
for idx in range(1, len(data_rows)):
    prev_err = data_rows[idx-1]['error']
    curr_err = data_rows[idx]['error']
    if prev_err * curr_err < 0 and abs(prev_err) >= 5 and abs(curr_err) >= 5:
        print(f"Sign reversal at seq {data_rows[idx]['seq']}: {prev_err} -> {curr_err} (time={data_rows[idx]['time_ms']}ms)")

print("\n\n" + "=" * 120)
print("SENSOR PATTERN ANALYSIS FOR INTERSECTIONS")
print("=" * 120)

# Find all cnt==8 frames
cnt8_frames = [(idx, data_rows[idx]) for idx in range(len(data_rows)) if data_rows[idx]['cnt'] == 8]
print(f"\nFound {len(cnt8_frames)} frames with cnt==8 (all sensors active)")
for idx, r in cnt8_frames:
    flags_val = int(r['flags'], 16)
    prev_err = data_rows[max(0, idx-1)]['error'] if idx > 0 else 'N/A'
    print(f"  seq={r['seq']}, time={r['time_ms']}, error={r['error']}, flags={r['flags']}, prev_error={prev_err}, turnOut={r['turnOut']}")

# Find all frames with both-side activation
print(f"\nBoth-side activation frames (left & right sensors both active):")
for idx, r in enumerate(data_rows):
    active_val = int(r['active'], 16)
    left_active = active_val & 0xC0
    right_active = active_val & 0x03
    cnt = r['cnt']
    if left_active and right_active and cnt >= 4:
        flags_val = int(r['flags'], 16)
        print(f"  seq={r['seq']}, time={r['time_ms']}, active={r['active']}, cnt={cnt}, error={r['error']}, flags={r['flags']}, turnOut={r['turnOut']}")
