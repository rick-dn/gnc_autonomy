#!/usr/bin/env python3
"""
Run the error-state EKF offline over a loaded log and score it.

Usage: python3 -m ga_est.run_filter DATA.npz [--predict-only] [--plot FILE.png]
                                            [--gate] [--gate-timeout S]

DATA.npz comes from ga_est.log_loader. The filter only reads the IMU, GPS and
baro arrays (plus the compass once, for the initial heading). Ground truth and
EKF3 are used afterwards for scoring.

--gate tests GPS position and GPS velocity separately and rejects either one
whose normalised innovation squared is above the 99.9% chi-square bound (off by
default, so the clean-flight results are unchanged). --gate-timeout S accepts
GPS position again after S seconds of continuous rejection by resetting the
position onto it, like EKF3 does.

The estimate is saved next to the input as *_est.npz (or *_est_gated.npz).
"""

import argparse
import os

import numpy as np

from ga_est.eskf import ErrorStateEKF, P_, V_, quat_from_euler, quat_to_euler


# Process noise - continuous-time densities
NOISE = {
    'acc': 0.3,          # m/s^2/sqrt(Hz)
    'gyro': 0.01,        # rad/s/sqrt(Hz)
    'acc_bias': 0.003,   # m/s^3/sqrt(Hz)
    'gyro_bias': 3e-4,   # rad/s^2/sqrt(Hz)
}

# Initial 1-sigma errors
INIT_STD = {
    'pos': 0.5,                                        # m
    'vel': 0.1,                                        # m/s
    'att': np.radians([2.0, 2.0, 5.0]),                # roll, pitch, yaw
    'acc_bias': 0.1,                                   # m/s^2
    'gyro_bias': 0.005,                                # rad/s
}

# Measurement noise, 1-sigma
GPS_POS_STD = np.array([0.05, 0.05, 0.1])   # m, N E D
GPS_VEL_STD = 0.05                          # m/s
BARO_STD = 0.05                             # m - clean SITL; use --baro-std for noisy logs

# Error-state indices the baro must not move: horizontal position and
# velocity. Without GPS the horizontal uncertainty grows to 100s of m, so even
# a small chance correlation with height turns every baro wiggle into metres
# of horizontal jump. With GPS present this changes nothing measurable.
BARO_FROZEN = [0, 1, 3, 4]

# Initial attitude from the first stretch of IMU/compass data (vehicle at rest)
INIT_WINDOW_S = 0.5

# Innovation gates, tested separately on GPS position and GPS velocity (like
# EKF3's SP and SV test ratios), so a position glitch doesn't also throw away
# good velocity: chi-square 99.9% bound for 3 degrees of freedom. A
# consistent filter exceeds it 1 time in 1000.
GPS_GATE_NIS = 16.27

# The logged IMU has been through ArduPilot's 20 Hz low-pass (INS_GYRO_FILTER,
# INS_ACCEL_FILTER), which delays it by ~1/(2*pi*20)*sqrt(2) = 11 ms. Treat
# each sample as measured this long before its timestamp. Found by shifting
# the estimate against truth: roll error is smallest at 10-15 ms.
IMU_DELAY_S = 0.012


def initial_attitude(data):
    """Roll/pitch from gravity in the accelerometer, yaw from the compass"""
    t0 = data['imu_t'][0]
    f = data['imu_acc'][data['imu_t'] < t0 + INIT_WINDOW_S].mean(axis=0)
    roll = np.arctan2(-f[1], -f[2])
    pitch = np.arctan2(f[0], np.hypot(f[1], f[2]))

    m = data['mag_field'][data['mag_t'] < t0 + INIT_WINDOW_S].mean(axis=0)
    # Tilt-compensate the field into the horizontal plane
    mx = (m[0] * np.cos(pitch) + m[1] * np.sin(roll) * np.sin(pitch)
          + m[2] * np.cos(roll) * np.sin(pitch))
    my = m[1] * np.cos(roll) - m[2] * np.sin(roll)
    yaw = np.arctan2(-my, mx) + float(data['mag_dec'])
    return roll, pitch, yaw


def run(data, predict_only=False, gate=False, gate_timeout=None, baro_std=BARO_STD):
    imu_t, gyro, acc = data['imu_t'] - IMU_DELAY_S, data['imu_gyro'], data['imu_acc']
    gps_t, gps_pos, gps_vel = data['gps_t'], data['gps_pos'], data['gps_vel']
    baro_t, baro_alt = data['baro_t'], data['baro_alt']
    lag = float(data['gps_lag_s'])

    roll, pitch, yaw = initial_attitude(data)
    first_gps = np.searchsorted(gps_t, imu_t[0])
    ekf = ErrorStateEKF(p=gps_pos[first_gps], v=np.zeros(3),
                        q=quat_from_euler(roll, pitch, yaw),
                        noise=NOISE, init_std=INIT_STD)

    n = len(imu_t)
    out = {'t': imu_t, 'pos': np.zeros((n, 3)), 'vel': np.zeros((n, 3)),
           'att': np.zeros((n, 3)), 'acc_bias': np.zeros((n, 3)),
           'gyro_bias': np.zeros((n, 3)), 'pos_std': np.zeros((n, 3)),
           'gps_t': [], 'gps_nis': [], 'gps_rejected': [], 'gps_vel_rejected': [],
           'gps_reset_t': []}
    rejecting_since = None

    H_gps = np.zeros((6, 15))
    H_gps[0:3, P_] = np.eye(3)
    H_gps[3:6, V_] = np.eye(3)
    R_gps = np.diag(np.concatenate([GPS_POS_STD ** 2, np.full(3, GPS_VEL_STD ** 2)]))
    H_baro = np.zeros((1, 15))
    H_baro[0, 2] = -1.0                  # baro measures height = -down
    R_baro = np.array([[baro_std ** 2]])

    gi = first_gps + 1
    bi = np.searchsorted(baro_t, imu_t[0], side='right')

    def store(k):
        out['pos'][k] = ekf.p
        out['vel'][k] = ekf.v
        out['att'][k] = quat_to_euler(ekf.q)
        out['acc_bias'][k] = ekf.ba
        out['gyro_bias'][k] = ekf.bg
        out['pos_std'][k] = np.sqrt(np.diag(ekf.P)[P_])

    store(0)
    for k in range(1, n):
        ekf.predict(gyro[k], acc[k], imu_t[k] - imu_t[k - 1])

        if not predict_only:
            # GPS: each fix describes the vehicle `lag` seconds before it was
            # logged, so compare it with the state stored at that time
            while gi < len(gps_t) and gps_t[gi] <= imu_t[k]:
                j = max(np.searchsorted(imu_t[:k], gps_t[gi] - lag) - 1, 0)
                residual = np.concatenate([gps_pos[gi] - out['pos'][j],
                                           gps_vel[gi] - out['vel'][j]])
                pos_nis = ekf.nis(residual[0:3], H_gps[0:3], R_gps[0:3, 0:3])
                vel_nis = ekf.nis(residual[3:6], H_gps[3:6], R_gps[3:6, 3:6])
                pos_ok = not (gate and pos_nis > GPS_GATE_NIS)
                vel_ok = not (gate and vel_nis > GPS_GATE_NIS)

                if pos_ok:
                    rejecting_since = None
                else:
                    rejecting_since = rejecting_since if rejecting_since is not None else gps_t[gi]
                    if gate_timeout is not None and gps_t[gi] - rejecting_since >= gate_timeout:
                        # Rejected too long - trust GPS position again. Shift
                        # the delayed fix forward by how far we've moved since.
                        ekf.reset_position(gps_pos[gi] + (ekf.p - out['pos'][j]),
                                           GPS_POS_STD ** 2)
                        out['gps_reset_t'].append(gps_t[gi])
                        rejecting_since = None

                # Fuse whatever passed in one update - with the gate off that's
                # the full 6-value fix, exactly as before
                keep = np.r_[np.full(3, pos_ok), np.full(3, vel_ok)]
                if keep.any():
                    ekf.correct(residual[keep], H_gps[keep], R_gps[np.ix_(keep, keep)])
                out['gps_t'].append(gps_t[gi])
                out['gps_nis'].append(pos_nis)
                out['gps_rejected'].append(not pos_ok)
                out['gps_vel_rejected'].append(not vel_ok)
                gi += 1

            while bi < len(baro_t) and baro_t[bi] <= imu_t[k]:
                residual = np.array([baro_alt[bi] - (-ekf.p[2])])
                ekf.correct(residual, H_baro, R_baro, frozen=BARO_FROZEN)
                bi += 1

        store(k)
    for key in ('gps_t', 'gps_nis', 'gps_rejected', 'gps_vel_rejected', 'gps_reset_t'):
        out[key] = np.array(out[key])
    return out


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def score(t, pos, vel, att_deg, data):
    """Errors against ground truth for any estimate (ours or EKF3)"""
    def interp(tq, ts, vals):
        return np.stack([np.interp(tq, ts, vals[:, i]) for i in range(3)], axis=-1)

    tp = interp(t, data['truth_t'], data['truth_pos'])
    tv = interp(t, data['truth_t'], data['truth_vel'])
    horiz = np.linalg.norm(pos[:, :2] - tp[:, :2], axis=1)
    vert = np.abs(pos[:, 2] - tp[:, 2])
    vel_err = np.linalg.norm(vel - tv, axis=1)

    truth_eul = np.degrees(np.array([quat_to_euler(q) for q in data['truth_quat']]))
    est_eul = interp(data['truth_att_t'], t, att_deg)
    att_err = np.abs(wrap_deg(est_eul - truth_eul))
    return {'horiz': horiz, 'vert': vert, 'vel': vel_err, 'att': att_err}


def report(label, s):
    print(f'{label:7} horizontal {s["horiz"].mean():7.3f} mean {s["horiz"].max():8.3f} max | '
          f'vertical {s["vert"].mean():7.3f} mean | velocity {s["vel"].mean():6.3f} m/s mean | '
          f'roll/pitch/yaw err {s["att"].mean(axis=0).round(2)} deg mean')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('data')
    parser.add_argument('--predict-only', action='store_true',
                        help='IMU propagation only, no GPS/baro corrections')
    parser.add_argument('--plot', help='save an error-vs-time plot here')
    parser.add_argument('--gate', action='store_true',
                        help='reject GPS fixes failing the chi-square innovation test')
    parser.add_argument('--gate-timeout', type=float, default=None,
                        help='with --gate: accept GPS again after this many seconds of rejection')
    parser.add_argument('--baro-std', type=float, default=BARO_STD,
                        help=f'baro noise the filter assumes, m (default {BARO_STD}; '
                             'the batch fault logs have 0.2 m of baro noise)')
    args = parser.parse_args()

    data = dict(np.load(args.data))
    est = run(data, predict_only=args.predict_only, gate=args.gate,
              gate_timeout=args.gate_timeout, baro_std=args.baro_std)
    ours = score(est['t'], est['pos'], est['vel'], np.degrees(est['att']), data)

    print('errors vs ground truth (m unless stated):')
    report('ours', ours)
    if not args.predict_only:
        ekf3 = score(data['ekf_t'], data['ekf_pos'], data['ekf_vel'], data['ekf_att'], data)
        report('EKF3', ekf3)
        print(f'final gyro bias  {est["gyro_bias"][-1].round(5)} rad/s')
        print(f'final accel bias {est["acc_bias"][-1].round(4)} m/s^2')
        nis = est['gps_nis']
        print(f'GPS position innovation: NIS median {np.median(nis):.2f} (3 dof expects ~2.4), '
              f'{np.mean(nis > GPS_GATE_NIS) * 100:.1f}% of fixes above the gate'
              + (f', {int(est["gps_rejected"].sum())} positions and '
                 f'{int(est["gps_vel_rejected"].sum())} velocities rejected, '
                 f'{len(est["gps_reset_t"])} resets' if args.gate else ''))
    else:
        t0 = est['t'][0]
        for secs in (1, 2, 5, 10, 20, 30, 45):
            k = np.searchsorted(est['t'], t0 + secs)
            if k < len(est['t']):
                print(f'  after {secs:2d} s: horizontal {ours["horiz"][k]:8.3f} m, '
                      f'vertical {ours["vert"][k]:8.3f} m, velocity {ours["vel"][k]:7.3f} m/s')

    suffix = '_predict' if args.predict_only else ('_est_gated' if args.gate else '_est')
    out_path = os.path.splitext(args.data)[0] + suffix + '.npz'
    np.savez_compressed(out_path, **est)
    print(f'saved {out_path}')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 3.5), dpi=120)
        ax.plot(est['t'] - est['t'][0], ours['horiz'], label='horizontal', color='#2a78d6')
        ax.plot(est['t'] - est['t'][0], ours['vert'], label='vertical', color='#eb6834')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('error vs truth (m)')
        if args.predict_only:
            ax.set_yscale('log')
        ax.legend(frameon=False)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot)
        print(f'plot {args.plot}')


if __name__ == '__main__':
    main()
