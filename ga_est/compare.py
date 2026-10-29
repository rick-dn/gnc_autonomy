#!/usr/bin/env python3
"""
Compare our estimator and EKF3 against ground truth: plots and a table.

Usage: python3 -m ga_est.compare DATA.npz [--out docs/img]

Needs DATA.npz (ga_est.log_loader), DATA_est.npz and DATA_predict.npz
(ga_est.run_filter, with and without --predict-only). Writes:
  est_position.png   horizontal position and velocity error vs time
  est_attitude.png   roll, pitch, yaw error vs time
  est_predict.png    IMU-only drift vs the full filter
and prints a markdown results table.
"""

import argparse
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from ga_est.eskf import quat_to_euler
from ga_est.run_filter import score, wrap_deg


OURS = '#2a78d6'
EKF3 = '#eb6834'
INK = '#0b0b0b'
INK_2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
SURFACE = '#fcfcfb'
AIRBORNE_FILL = '#f0efec'

# Truth altitude above this counts as airborne (for shading and the table)
AIRBORNE_ALT_M = 0.3


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def airborne_window(data):
    alt = -data['truth_pos'][:, 2]
    up = data['truth_t'][alt > alt[:400].mean() + AIRBORNE_ALT_M]
    return up[0], up[-1]


def shade(ax, t0, window):
    ax.axvspan(window[0] - t0, window[1] - t0, color=AIRBORNE_FILL, zorder=0)


def signed_att_error(t, att_deg, data):
    truth = np.degrees(np.array([quat_to_euler(q) for q in data['truth_quat']]))
    est = np.stack([np.interp(data['truth_att_t'], t, att_deg[:, i]) for i in range(3)], -1)
    return wrap_deg(est - truth)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('data')
    parser.add_argument('--out', default='docs/img')
    args = parser.parse_args()

    base = os.path.splitext(args.data)[0]
    data = dict(np.load(args.data))
    est = dict(np.load(base + '_est.npz'))
    pred = dict(np.load(base + '_predict.npz'))

    ours = score(est['t'], est['pos'], est['vel'], np.degrees(est['att']), data)
    ekf3 = score(data['ekf_t'], data['ekf_pos'], data['ekf_vel'], data['ekf_att'], data)
    imu_only = score(pred['t'], pred['pos'], pred['vel'], np.degrees(pred['att']), data)

    window = airborne_window(data)
    t0 = data['imu_t'][0]

    # --- table, airborne part of the flight only --------------------------
    def in_air(t):
        return (t >= window[0]) & (t <= window[1])

    def row(label, s, t):
        m = in_air(t)
        ma = in_air(data['truth_att_t'])
        att = s['att'][ma].mean(axis=0)
        return (f'| {label} | {s["horiz"][m].mean() * 100:.1f} cm | '
                f'{s["horiz"][m].max() * 100:.1f} cm | {s["vert"][m].mean() * 100:.1f} cm | '
                f'{s["vel"][m].mean() * 100:.1f} cm/s | '
                f'{att[0]:.2f}° / {att[1]:.2f}° / {att[2]:.2f}° |')

    print(f'airborne {window[0] - t0:.1f}-{window[1] - t0:.1f} s after log start\n')
    print('| | Horizontal (mean) | Horizontal (max) | Vertical (mean) | Velocity (mean) '
          '| Roll / pitch / yaw (mean) |')
    print('|---|---|---|---|---|---|')
    print(row('Ours', ours, est['t']))
    print(row('EKF3', ekf3, data['ekf_t']))

    os.makedirs(args.out, exist_ok=True)
    plt.rcParams['font.family'] = 'DejaVu Sans'

    # --- position and velocity error ---------------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(8, 5.2), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for ax, key, ylabel, scale in ((axes[0], 'horiz', 'horizontal position\nerror (cm)', 100),
                                   (axes[1], 'vel', 'velocity error\n(cm/s)', 100)):
        style(ax)
        shade(ax, t0, window)
        ax.plot(data['ekf_t'] - t0, ekf3[key] * scale, color=EKF3, linewidth=1.5, label='EKF3')
        ax.plot(est['t'] - t0, ours[key] * scale, color=OURS, linewidth=1.5, label='ours')
        ax.set_ylabel(ylabel, color=INK_2, fontsize=9)
        ax.set_ylim(bottom=0)
    axes[0].set_title('Error vs ground truth (shaded: airborne)', color=INK, fontsize=11, loc='left')
    axes[0].legend(frameon=False, fontsize=9, labelcolor=INK_2, loc='upper right')
    axes[1].set_xlabel('time since arming (s)', color=INK_2, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'est_position.png'), facecolor=SURFACE)

    # --- attitude error -------------------------------------------------------
    ours_att = signed_att_error(est['t'], np.degrees(est['att']), data)
    ekf3_att = signed_att_error(data['ekf_t'], data['ekf_att'], data)
    ta = data['truth_att_t'] - t0
    fig, axes = plt.subplots(3, 1, figsize=(8, 6), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for i, (ax, name) in enumerate(zip(axes, ('roll', 'pitch', 'yaw'))):
        style(ax)
        shade(ax, t0, window)
        ax.axhline(0, color=MUTED, linewidth=0.8)
        ax.plot(ta, ekf3_att[:, i], color=EKF3, linewidth=1.5, label='EKF3')
        ax.plot(ta, ours_att[:, i], color=OURS, linewidth=1.5, label='ours')
        ax.set_ylabel(f'{name} error (°)', color=INK_2, fontsize=9)
    axes[0].set_title('Attitude error vs ground truth (shaded: airborne)', color=INK,
                      fontsize=11, loc='left')
    axes[0].legend(frameon=False, fontsize=9, labelcolor=INK_2, loc='upper right')
    axes[2].set_xlabel('time since arming (s)', color=INK_2, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'est_attitude.png'), facecolor=SURFACE)

    # --- IMU-only drift vs full filter -----------------------------------------
    fig, ax = plt.subplots(figsize=(8, 3.4), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style(ax)
    ax.plot(pred['t'] - t0, imu_only['horiz'], color=MUTED, linewidth=1.5)
    ax.plot(est['t'] - t0, ours['horiz'], color=OURS, linewidth=1.5)
    ax.set_yscale('log')
    k = np.searchsorted(pred['t'], t0 + 30)
    ax.annotate('IMU only', (pred['t'][k] - t0, imu_only['horiz'][k]), xytext=(6, 6),
                textcoords='offset points', color=INK_2, fontsize=9)
    ax.annotate('with GPS + baro', (est['t'][k] - t0, ours['horiz'][k]), xytext=(6, 14),
                textcoords='offset points', color=INK_2, fontsize=9)
    ax.set_xlabel('time since arming (s)', color=INK_2, fontsize=9)
    ax.set_ylabel('horizontal error (m, log)', color=INK_2, fontsize=9)
    ax.set_title('What the corrections do', color=INK, fontsize=11, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'est_predict.png'), facecolor=SURFACE)

    print(f'\nplots written to {args.out}/')


if __name__ == '__main__':
    main()
