#!/usr/bin/env python3
"""
P1 GPS-loss plots from an ArduPilot SITL dataflash log.

Usage: python3 tools/eval/p1_gps_loss_plots.py LOG.BIN [--out docs/img]

Finds the GPS fault (SIM_GPS1_ENABLE changes) and the EKF source-set switches
(events 85/86) in the log, compares the EKF position (XKF1, core 0) with the
simulator's true position (SIM2), prints a summary and writes two plots:
  p1_error.png  horizontal position error vs time
  p1_track.png  top-down true vs estimated track
"""

import argparse
import bisect
import math
import os
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pymavlink import mavutil


# ArduPilot LogEvent ids for EKF source set selection
EV_SOURCE_SET_1 = 85
EV_SOURCE_SET_2 = 86

# Colours - validated categorical pair, plus chart chrome
TRUE_COLOUR = '#2a78d6'
EKF_COLOUR = '#eb6834'
INK = '#0b0b0b'
INK_2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
SURFACE = '#fcfcfb'
OUTAGE_FILL = '#f0efec'


def read_log(path):
    log = mavutil.mavlink_connection(path)
    sim, ekf, gps_changes, switches = [], [], [], []
    gps_enable = None
    while True:
        msg = log.recv_match(type=['SIM2', 'XKF1', 'PARM', 'EV'])
        if msg is None:
            break
        t = msg.TimeUS / 1e6
        kind = msg.get_type()
        if kind == 'SIM2':
            sim.append((t, msg.PN, msg.PE))
        elif kind == 'XKF1' and msg.C == 0:
            ekf.append((t, msg.PN, msg.PE))
        elif kind == 'PARM' and msg.Name == 'SIM_GPS1_ENABLE':
            if gps_enable is not None and msg.Value != gps_enable:
                gps_changes.append((t, int(msg.Value)))
            gps_enable = msg.Value
        elif kind == 'EV' and msg.Id in (EV_SOURCE_SET_1, EV_SOURCE_SET_2):
            switches.append((t, 1 if msg.Id == EV_SOURCE_SET_1 else 2))
    return sim, ekf, gps_changes, switches


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('log')
    parser.add_argument('--out', default='docs/img')
    args = parser.parse_args()

    sim, ekf, gps_changes, switches = read_log(args.log)
    gps_off = next(t for t, v in gps_changes if v == 0)
    gps_on = next(t for t, v in gps_changes if v == 1 and t > gps_off)
    to_flow = next(t for t, s in switches if s == 2 and t > gps_off)
    to_gps = next(t for t, s in switches if s == 1 and t > gps_on)

    # Pair each true sample with the nearest EKF sample
    ekf_t = [e[0] for e in ekf]
    pairs = []
    for t, n, e in sim:
        i = min(bisect.bisect_left(ekf_t, t), len(ekf) - 1)
        pairs.append((t, n, e, ekf[i][1], ekf[i][2]))

    # SIM2 is relative to home, XKF1 to the EKF origin - remove the constant
    # offset using the 20 s of GPS flight before the fault
    pre = [p for p in pairs if gps_off - 20 < p[0] < gps_off]
    off_n = statistics.mean(p[1] - p[3] for p in pre)
    off_e = statistics.mean(p[2] - p[4] for p in pre)

    def error(p):
        return math.hypot(p[1] - p[3] - off_n, p[2] - p[4] - off_e)

    window = [p for p in pairs if gps_off - 15 < p[0] < to_gps + 15]
    outage = [p for p in pairs if to_flow < p[0] < gps_on]
    start = next(p for p in pairs if p[0] > gps_off)
    end = next(p for p in pairs if p[0] > gps_on)

    print(f'GPS off -> flow:  {to_flow - gps_off:.1f} s')
    print(f'GPS on  -> GPS:   {to_gps - gps_on:.1f} s')
    print(f'Outage on flow:   {gps_on - to_flow:.1f} s')
    print(f'Error on flow:    mean {statistics.mean(map(error, outage)):.2f} m, '
          f'max {max(map(error, outage)):.2f} m')
    print(f'True drift:       {math.hypot(end[1] - start[1], end[2] - start[2]):.2f} m')
    print(f'EKF believed:     {math.hypot(end[3] - start[3], end[4] - start[4]):.2f} m')

    os.makedirs(args.out, exist_ok=True)
    plt.rcParams['font.family'] = 'DejaVu Sans'

    # Plot 1 - error vs time, t = 0 at GPS loss
    fig, ax = plt.subplots(figsize=(8, 3.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style(ax)
    ax.axvspan(0, gps_on - gps_off, color=OUTAGE_FILL, zorder=0)
    ax.text((gps_on - gps_off) / 2, 0.02, 'GPS off', ha='center', va='bottom',
            color=INK_2, fontsize=9, transform=ax.get_xaxis_transform())
    ax.plot([p[0] - gps_off for p in window], [error(p) for p in window],
            color=TRUE_COLOUR, linewidth=2)
    for t, label in ((to_flow, 'switched to flow'), (to_gps, 'back to GPS')):
        ax.axvline(t - gps_off, color=MUTED, linewidth=1, linestyle='--')
        ax.text(t - gps_off, 0.97, f' {label}', color=INK_2, fontsize=9,
                va='top', transform=ax.get_xaxis_transform())
    ax.set_xlabel('time since GPS loss (s)', color=INK_2, fontsize=9)
    ax.set_ylabel('position error (m)', color=INK_2, fontsize=9)
    ax.set_title('EKF horizontal position error vs ground truth', color=INK,
                 fontsize=11, loc='left')
    ax.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'p1_error.png'), facecolor=SURFACE)

    # Plot 2 - top-down track from GPS loss until GPS returns, relative to
    # where GPS was lost. Stops before the switch back so the EKF's correction
    # jump doesn't draw across the plot.
    track = [p for p in pairs if gps_off < p[0] < gps_on]
    n0, e0 = start[1], start[2]
    fig, ax = plt.subplots(figsize=(5, 5), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style(ax)
    ax.plot([p[2] - e0 for p in track], [p[1] - n0 for p in track],
            color=TRUE_COLOUR, linewidth=2, label='true position')
    ax.plot([p[4] + off_e - e0 for p in track], [p[3] + off_n - n0 for p in track],
            color=EKF_COLOUR, linewidth=2, label='EKF estimate')
    ax.plot(0, 0, 'o', color=INK, markersize=8, markeredgecolor=SURFACE,
            markeredgewidth=2)
    ax.annotate('GPS lost here', (0, 0), xytext=(0, -46),
                textcoords='offset points', ha='center', color=INK_2, fontsize=9)
    last = track[-1]
    ax.plot(last[2] - e0, last[1] - n0, 'o', color=TRUE_COLOUR, markersize=8,
            markeredgecolor=SURFACE, markeredgewidth=2)
    ax.annotate(f'true position when GPS\ncame back: '
                f'{math.hypot(last[1] - n0, last[2] - e0):.2f} m away',
                (last[2] - e0, last[1] - n0), xytext=(0, -34),
                textcoords='offset points', ha='center', color=INK_2, fontsize=9)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('east (m)', color=INK_2, fontsize=9)
    ax.set_ylabel('north (m)', color=INK_2, fontsize=9)
    ax.set_title('Track while on optical flow', color=INK, fontsize=11, loc='left')
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_2, loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'p1_track.png'), facecolor=SURFACE)

    print(f'Plots written to {args.out}/')


if __name__ == '__main__':
    main()
