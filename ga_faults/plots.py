#!/usr/bin/env python3
"""
Plots for the fault matrix write-up, from single batch runs.

Usage: python3 -m ga_faults.plots [--runs-dir runs/faults] [--out docs/img]

  fm_glitch.png      GPS glitch: EKF3 position error and its position test
                     ratio (SP) - rejection, then the reset onto bad GPS
  fm_lane_switch.png large accel bias on IMU 1: error of each EKF3 core and
                     of the position actually used, around the lane switch
"""

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pymavlink import mavutil

from ga_est.log_loader import lla_to_ned


ROOT = Path(__file__).resolve().parents[1]

BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
INK, INK_2, MUTED, GRID, SURFACE, FAULT_FILL = ('#0b0b0b', '#52514e', '#898781',
                                                '#e1e0d9', '#fcfcfb', '#f0efec')


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def read_run(run_dir):
    record = json.loads((run_dir / 'run.json').read_text())
    log = mavutil.mavlink_connection(str(run_dir / record['log']))
    out = {'POS': [], 'SIM': [], 'XKF1': {0: [], 1: []}, 'XKF4': [], 'MSG': [], 'origin': None}
    while True:
        msg = log.recv_match(type=['POS', 'SIM', 'XKF1', 'XKF4', 'MSG', 'ORGN'])
        if msg is None:
            break
        t = msg.TimeUS / 1e6
        kind = msg.get_type()
        if kind == 'ORGN' and msg.Type == 0 and out['origin'] is None:
            out['origin'] = (msg.Lat, msg.Lng, msg.Alt)
        elif kind in ('POS', 'SIM'):
            out[kind].append((t, msg.Lat, msg.Lng, msg.Alt))
        elif kind == 'XKF1' and msg.C in (0, 1):
            out['XKF1'][msg.C].append((t, msg.PN, msg.PE))
        elif kind == 'XKF4' and msg.C == 0:
            out['XKF4'].append((t, msg.SP))
        elif kind == 'MSG':
            out['MSG'].append((t, msg.Message))
    return record, out


def truth_ne(out):
    sim = np.array(out['SIM'])
    ned = lla_to_ned(sim[:, 1], sim[:, 2], sim[:, 3], out['origin'])
    return sim[:, 0], ned[:, 0], ned[:, 1]


def error_vs_truth(t, n, e, truth):
    tt, tn, te = truth
    return np.hypot(n - np.interp(t, tt, tn), e - np.interp(t, tt, te))


def pos_error(out, truth):
    pos = np.array(out['POS'])
    ned = lla_to_ned(pos[:, 1], pos[:, 2], pos[:, 3], out['origin'])
    return pos[:, 0], error_vs_truth(pos[:, 0], ned[:, 0], ned[:, 1], truth)


def glitch_plot(run_dir, path):
    record, out = read_run(run_dir)
    inject = record['events']['inject']['sent_s']
    clear = record['events']['clear']['sent_s']
    t, err = pos_error(out, truth_ne(out))
    xk = np.array(out['XKF4'])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5), dpi=150, sharex=True,
                                   gridspec_kw={'height_ratios': [3, 2]})
    fig.patch.set_facecolor(SURFACE)
    window = (t > inject - 10) & (t < clear + 20)
    for ax in (ax1, ax2):
        style(ax)
        ax.axvspan(0, clear - inject, color=FAULT_FILL, zorder=0)
    ax1.plot(t[window] - inject, err[window], color=ORANGE, linewidth=1.8)
    ax1.set_ylabel('position error\nvs truth (m)', color=INK_2, fontsize=9)
    ax1.set_ylim(bottom=0)
    ax1.set_title('GPS glitch (5 m), EKF3: rejected for 10 s, then accepted',
                  color=INK, fontsize=11, loc='left')
    ax1.text((clear - inject) / 2, 0.5, 'glitch active', ha='center', va='center',
             color=INK_2, fontsize=9, transform=ax1.get_xaxis_transform())

    xw = (xk[:, 0] > inject - 10) & (xk[:, 0] < clear + 20)
    ax2.plot(xk[xw, 0] - inject, xk[xw, 1], color=ORANGE, linewidth=1.8)
    ax2.axhline(1.0, color=MUTED, linewidth=1, linestyle='--')
    ax2.text(clear - inject + 19, 1.05, 'reject above 1', ha='right', va='bottom',
             color=INK_2, fontsize=8)
    ax2.set_ylabel('position test\nratio (SP)', color=INK_2, fontsize=9)
    ax2.set_xlabel('time since glitch injected (s)', color=INK_2, fontsize=9)
    ax2.set_ylim(0, max(1.6, xk[xw, 1].max() * 1.1))
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    print(f'plot {path}')


def lane_switch_plot(run_dir, path):
    record, out = read_run(run_dir)
    inject = record['events']['inject']['sent_s']
    truth = truth_ne(out)
    switch = next((t for t, m in out['MSG'] if 'lane switch' in m and t >= inject), None)

    fig, ax = plt.subplots(figsize=(8, 3.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style(ax)
    lo, hi = inject - 5, inject + 25
    series = [('core 0 (IMU 1, faulty)', np.array(out['XKF1'][0]), AQUA),
              ('core 1 (IMU 2)', np.array(out['XKF1'][1]), BLUE)]
    for label, x, colour in series:
        err = error_vs_truth(x[:, 0], x[:, 1], x[:, 2], truth)
        w = (x[:, 0] > lo) & (x[:, 0] < hi)
        ax.plot(x[w, 0] - inject, err[w], color=colour, linewidth=1.6, label=label)
    t, err = pos_error(out, truth)
    w = (t > lo) & (t < hi)
    ax.plot(t[w] - inject, err[w], color=ORANGE, linewidth=2.2, linestyle='--',
            label='position actually used')
    ax.axvline(0, color=MUTED, linewidth=1)
    ax.text(0, 0.97, ' bias injected', color=INK_2, fontsize=9, va='top',
            transform=ax.get_xaxis_transform())
    if switch is not None:
        ax.axvline(switch - inject, color=MUTED, linewidth=1, linestyle='--')
        ax.text(switch - inject, 0.85, f' lane switch (+{switch - inject:.1f} s)',
                color=INK_2, fontsize=9, va='top', transform=ax.get_xaxis_transform())
    ax.set_xlabel('time since bias injected (s)', color=INK_2, fontsize=9)
    ax.set_ylabel('position error vs truth (m)', color=INK_2, fontsize=9)
    ax.set_ylim(bottom=0)
    ax.set_title('Accelerometer bias 2 m/s² on IMU 1: EKF3 lane switch',
                 color=INK, fontsize=11, loc='left')
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_2, loc='upper right')
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    print(f'plot {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--runs-dir', default=str(ROOT / 'runs/faults'))
    parser.add_argument('--out', default=str(ROOT / 'docs/img'))
    args = parser.parse_args()
    runs, out = Path(args.runs_dir), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    glitch_plot(runs / 'gps_glitch_5m/run_1', out / 'fm_glitch.png')
    lane_switch_plot(runs / 'accel_bias_large/run_1', out / 'fm_lane_switch.png')


if __name__ == '__main__':
    main()
