#!/usr/bin/env python3
"""
Fault matrix - detection time, fallback and position error per fault, from
the batch runs in runs/faults/<fault>/run_<k>/.

Usage: python3 -m ga_faults.matrix [--runs-dir runs/faults] [--csv FILE]

Per run, from run.json (inject/clear times) and the dataflash log:
  detection   first EKF reaction after the fault goes in: an XKF4 test ratio
              (SV, SP, SH, SM) on core 0 above 1, or an EKF/source message
  fallback    what the vehicle did, classified from its status messages
  error       horizontal error of the position ArduPilot used (POS) against
              SITL ground truth (SIM): baseline before the fault, during it,
              and at the end of the run after it was cleared
Then aggregated over the runs of each fault into a markdown table.
"""

import argparse
import csv
import json
import statistics
from collections import Counter
from pathlib import Path

import numpy as np
from pymavlink import mavutil

from ga_est.log_loader import lla_to_ned


ROOT = Path(__file__).resolve().parents[1]

# Status message -> fallback label, in the order they're reported
FALLBACKS = [
    ('Auto switched to Source 2', 'switched to optical flow'),
    ('GPS Glitch', 'rejected GPS'),
    ('stopped aiding', 'lost position aiding'),
    ('lane switch', 'lane switch'),
    ('switching to compass', 'compass switch'),
    ('EKF Failsafe', 'EKF failsafe'),
]

TEST_RATIOS = ('SV', 'SP', 'SH', 'SM')

BASELINE_S = 20.0        # error baseline: this long before the fault
RECOVERED_LAST_S = 10.0  # "recovered" if the last 10 s are back near baseline
RECOVERED_MARGIN_M = 0.5


def read_log(path):
    log = mavutil.mavlink_connection(str(path))
    pos, sim, xkf4, msgs = [], [], [], []
    while True:
        msg = log.recv_match(type=['POS', 'SIM', 'XKF4', 'MSG'])
        if msg is None:
            break
        t = msg.TimeUS / 1e6
        kind = msg.get_type()
        if kind == 'POS':
            pos.append((t, msg.Lat, msg.Lng, msg.Alt))
        elif kind == 'SIM':
            sim.append((t, msg.Lat, msg.Lng, msg.Alt))
        elif kind == 'XKF4' and msg.C == 0:
            xkf4.append((t,) + tuple(getattr(msg, r) for r in TEST_RATIOS))
        elif kind == 'MSG':
            msgs.append((t, msg.Message))
    return np.array(pos), np.array(sim), np.array(xkf4), msgs


def horizontal_error(pos, sim):
    """Error of the used estimate vs truth, sampled at the POS times"""
    origin = tuple(sim[0, 1:4])
    truth = lla_to_ned(sim[:, 1], sim[:, 2], sim[:, 3], origin)
    est = lla_to_ned(pos[:, 1], pos[:, 2], pos[:, 3], origin)
    tn = np.interp(pos[:, 0], sim[:, 0], truth[:, 0])
    te = np.interp(pos[:, 0], sim[:, 0], truth[:, 1])
    return pos[:, 0], np.hypot(est[:, 0] - tn, est[:, 1] - te)


def analyse_run(run_dir):
    record = json.loads((run_dir / 'run.json').read_text())
    pos, sim, xkf4, msgs = read_log(run_dir / record['log'])
    t_err, err = horizontal_error(pos, sim)
    events = record['events']
    takeoff = events['takeoff']['sent_s']
    inject = events.get('inject', {}).get('sent_s')
    clear = events.get('clear', {}).get('sent_s')
    end = t_err[-1]
    if inject is None:
        # Baseline run: score the same window a fault would have used
        inject = takeoff + record['fault_after_takeoff_s']
        clear = inject + 60.0

    def window(a, b):
        sel = err[(t_err >= a) & (t_err < b)]
        return sel if len(sel) else np.array([np.nan])

    row = {'fault': record['fault'], 'run': run_dir.name}

    # Detection: earliest test ratio > 1 or relevant message after the fault
    candidates = []
    after = xkf4[xkf4[:, 0] >= inject]
    for i, name in enumerate(TEST_RATIOS, start=1):
        over = after[after[:, i] > 1.0]
        if len(over):
            candidates.append((over[0, 0], f'{name} > 1'))
    fallbacks = []
    for t, text in msgs:
        if inject <= t < clear:
            for pattern, label in FALLBACKS:
                if pattern in text:
                    candidates.append((t, text))
                    if label not in fallbacks:
                        fallbacks.append(label)
    if record['events'].get('inject') and candidates:
        t_det, how = min(candidates)
        row['detected'] = True
        row['detection_s'] = round(t_det - inject, 2)
        row['detected_by'] = how
    else:
        row['detected'] = False
        row['detection_s'] = None
        row['detected_by'] = ''
    if 'disarmed_early' in events:
        fallbacks.append('landed')
    row['fallback'] = ', '.join(fallbacks) if fallbacks else 'none'

    baseline = window(inject - BASELINE_S, inject)
    during = window(inject, clear)
    last = window(end - RECOVERED_LAST_S, end)
    row['err_baseline_m'] = round(float(np.nanmean(baseline)), 3)
    row['err_mean_m'] = round(float(np.nanmean(during)), 3)
    row['err_max_m'] = round(float(np.nanmax(during)), 3)
    row['err_end_m'] = round(float(np.nanmean(last)), 3)
    row['recovered'] = bool(row['err_end_m'] <= row['err_baseline_m'] + RECOVERED_MARGIN_M)
    return row


def summarise(rows):
    """One markdown row per fault, aggregated over its runs"""
    from ga_faults.faults import FAULTS
    lines = ['| Fault | Runs | Detected | Detection time | Detected by | Fallback | '
             'Error during fault (mean / max) | Recovered |',
             '|---|---|---|---|---|---|---|---|']
    for fault in FAULTS:
        rs = [r for r in rows if r['fault'] == fault]
        if not rs:
            continue
        det = [r for r in rs if r['detected']]
        if det:
            times = [r['detection_s'] for r in det]
            dt = (f'{statistics.mean(times):.1f} s ({min(times):.1f}–{max(times):.1f})'
                  if len(times) > 1 else f'{times[0]:.1f} s')
            by = Counter(r['detected_by'].split(':')[0] for r in det).most_common(1)[0][0]
        else:
            dt, by = '—', '—'
        fallback = Counter(r['fallback'] for r in rs).most_common(1)[0][0]
        err_mean = statistics.mean(r['err_mean_m'] for r in rs)
        err_max = max(r['err_max_m'] for r in rs)
        rec = sum(r['recovered'] for r in rs)
        lines.append(f'| {FAULTS[fault]["description"]} | {len(rs)} | {len(det)}/{len(rs)} | '
                     f'{dt} | {by} | {fallback} | {err_mean:.2f} / {err_max:.2f} m | '
                     f'{rec}/{len(rs)} |')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--runs-dir', default=str(ROOT / 'runs/faults'))
    parser.add_argument('--csv', default=None, help='per-run results (default: runs-dir/matrix.csv)')
    args = parser.parse_args()

    runs_dir = Path(args.runs_dir)
    rows = []
    for run_json in sorted(runs_dir.glob('*/run_*/run.json')):
        record = json.loads(run_json.read_text())
        if not record.get('completed') or not record.get('log'):
            print(f'skipping {run_json.parent} (incomplete)')
            continue
        rows.append(analyse_run(run_json.parent))

    out_csv = Path(args.csv) if args.csv else runs_dir / 'matrix.csv'
    with open(out_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    table = summarise(rows)
    (runs_dir / 'matrix.md').write_text(table + '\n')
    print(table)
    print(f'\nper-run results: {out_csv}')


if __name__ == '__main__':
    main()
