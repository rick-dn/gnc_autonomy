#!/usr/bin/env python3
"""
Run the custom estimator on batch fault runs and compare it with EKF3.

Usage:
  python3 -m ga_est.compare_faults runs/faults/gps_glitch_5m/run_1 [more run dirs]
                                   [--gate-timeout 10] [--baro-std 0.25] [--plot FILE.png]

For each run directory made by ga_faults.run (run.json + logs/*.BIN) this:
  1. loads the log with ga_est.log_loader (cached as data.npz in the run dir)
  2. runs the filter three ways: no gate, gated, gated with a reset timeout
  3. scores each, and the position EKF3 actually used (POS), against ground
     truth during the fault and after it clears
The table is printed; --plot draws error vs time for the first run given.

Note for GPS-loss runs: the custom filter has no optical flow, so during the
outage it is IMU-only while EKF3 is on flow - compare drift, not a contest.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from ga_est.log_loader import load_log
from ga_est.run_filter import run, score


VARIANTS = [
    ('ours, no gate', {'gate': False}),
    ('ours, gated', {'gate': True}),
    ('ours, gated + reset', {'gate': True, 'reset': True}),
]

COLOURS = {'ours, gated': '#2a78d6', 'EKF3 (POS)': '#eb6834',
           'ours, no gate': '#1baf7a', 'ours, gated + reset': '#eda100'}
INK, INK_2, MUTED, GRID, SURFACE, FAULT_FILL = ('#0b0b0b', '#52514e', '#898781',
                                                '#e1e0d9', '#fcfcfb', '#f0efec')

AFTER_CLEAR_S = 15.0     # score this long after the fault is cleared


def load_run(run_dir):
    record = json.loads((run_dir / 'run.json').read_text())
    cache = run_dir / 'data.npz'
    if cache.exists():
        data = dict(np.load(cache))
    else:
        data = load_log(str(run_dir / record['log']))
        np.savez_compressed(cache, **data)
    return record, data


def horizontal_error(t, pos, data):
    tn = np.interp(t, data['truth_t'], data['truth_pos'][:, 0])
    te = np.interp(t, data['truth_t'], data['truth_pos'][:, 1])
    return np.hypot(pos[:, 0] - tn, pos[:, 1] - te)


def analyse(run_dir, gate_timeout, baro_std):
    record, data = load_run(run_dir)
    inject = record['events']['inject']['sent_s']
    clear = record['events']['clear']['sent_s']

    series = {}
    stats = {}
    for label, opts in VARIANTS:
        est = run(data, gate=opts['gate'], baro_std=baro_std,
                  gate_timeout=gate_timeout if opts.get('reset') else None)
        series[label] = (est['t'], horizontal_error(est['t'], est['pos'], data))
        in_fault = (est['gps_t'] >= inject) & (est['gps_t'] < clear)
        stats[label] = {
            'rejected': int(est['gps_rejected'][in_fault].sum()),
            'fixes': int(in_fault.sum()),
            'resets': len(est['gps_reset_t']),
        }
    series['EKF3 (POS)'] = (data['used_t'], horizontal_error(data['used_t'], data['used_pos'], data))

    rows = {}
    for label, (t, err) in series.items():
        during = err[(t >= inject) & (t < clear)]
        after = err[(t >= clear) & (t < clear + AFTER_CLEAR_S)]
        rows[label] = {'during_mean': during.mean(), 'during_max': during.max(),
                       'after_mean': after.mean(), **stats.get(label, {})}
    return record, inject, clear, series, rows


def print_table(name, rows, gate_timeout):
    print(f'\n{name}  (horizontal error vs truth, m; reset timeout {gate_timeout} s)')
    print('| | During fault (mean) | During fault (max) | 15 s after clear (mean) '
          '| GPS fixes rejected in fault | Resets |')
    print('|---|---|---|---|---|---|')
    for label, r in rows.items():
        rej = f'{r["rejected"]}/{r["fixes"]}' if 'rejected' in r else '—'
        resets = r.get('resets', '—')
        print(f'| {label} | {r["during_mean"]:.2f} | {r["during_max"]:.2f} | '
              f'{r["after_mean"]:.2f} | {rej} | {resets} |')


def plot(path, title, inject, clear, series):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 3.8), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)

    ax.axvspan(0, clear - inject, color=FAULT_FILL, zorder=0)
    ax.text((clear - inject) / 2, 0.97, 'fault active', ha='center', va='top',
            color=INK_2, fontsize=9, transform=ax.get_xaxis_transform())
    for label in ('ours, no gate', 'ours, gated + reset', 'EKF3 (POS)', 'ours, gated'):
        t, err = series[label]
        sel = (t >= inject - 10) & (t < clear + AFTER_CLEAR_S)
        ax.plot(t[sel] - inject, err[sel], color=COLOURS[label], linewidth=1.6, label=label)
    ax.set_xlabel('time since fault injected (s)', color=INK_2, fontsize=9)
    ax.set_ylabel('horizontal error vs truth (m)', color=INK_2, fontsize=9)
    ax.set_ylim(bottom=0)
    ax.set_title(title, color=INK, fontsize=11, loc='left')
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_2, loc='center right')
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    print(f'plot {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('run_dirs', nargs='+')
    parser.add_argument('--gate-timeout', type=float, default=10.0,
                        help='seconds of rejection before the gated+reset variant accepts GPS')
    parser.add_argument('--baro-std', type=float, default=0.25,
                        help='baro noise the filter assumes, m (batch logs have 0.2 m)')
    parser.add_argument('--plot', help='error-vs-time plot for the first run dir')
    args = parser.parse_args()

    for i, run_dir in enumerate(Path(d) for d in args.run_dirs):
        record, inject, clear, series, rows = analyse(run_dir, args.gate_timeout, args.baro_std)
        print_table(f'{record["fault"]} / {run_dir.name}: {record["description"]}',
                    rows, args.gate_timeout)
        if args.plot and i == 0:
            plot(args.plot, f'{record["description"]}: estimator vs EKF3',
                 inject, clear, series)


if __name__ == '__main__':
    main()
