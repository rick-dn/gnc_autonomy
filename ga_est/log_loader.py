#!/usr/bin/env python3
"""
Log loader - ArduPilot SITL dataflash log -> time-aligned numpy arrays.

Usage: python3 -m ga_est.log_loader LOG.BIN [--out FILE.npz]

Everything comes out in one local NED frame centred on the EKF origin (ORGN),
with time in seconds from boot (TimeUS / 1e6):

  imu_t, imu_gyro, imu_acc       IMU 0, body FRD, rad/s and m/s^2 (400 Hz with
                                 Fast IMU logging)
  gps_t, gps_pos, gps_vel        GPS 0 with a 3D fix, NED m and m/s
  gps_nsats, gps_hdop
  baro_t, baro_alt               baro 0, height above the boot reading, m (up)
  mag_t, mag_field, mag_dec      compass 0, body FRD mGauss, and COMPASS_DEC (rad).
                                 Only used to set the initial heading.
  truth_t, truth_pos, truth_vel  SITL ground truth (SIM2), NED m and m/s
  truth_att_t, truth_quat        SITL attitude (SIM), body->NED quaternion w,x,y,z
  ekf_t, ekf_pos, ekf_vel        EKF3 core 0 (XKF1), NED m and m/s
  ekf_att                        EKF3 roll, pitch, yaw in deg
  used_t, used_pos               POS: the position ArduPilot actually used, NED m
                                 (follows lane switches, unlike XKF1 core 0)
  origin                         EKF origin lat, lon (deg), alt (m AMSL)
  gps_lag_s                      SIM_GPS1_LAG_MS: how old each GPS fix is

The estimator should not read truth_* or ekf_* - they are for scoring only.
"""

import argparse
import math
import os

import numpy as np
from pymavlink import mavutil


# WGS-84
EARTH_A = 6378137.0
EARTH_E2 = 6.69437999014e-3

GPS_FIX_3D = 3


def lla_to_ned(lat, lon, alt, origin):
    """Flat-earth lat/lon/alt -> NED metres about origin. Fine over a few km."""
    lat0, lon0, alt0 = origin
    phi = math.radians(lat0)
    s = math.sin(phi)
    r_n = EARTH_A * (1 - EARTH_E2) / (1 - EARTH_E2 * s * s) ** 1.5   # meridian
    r_e = EARTH_A / math.sqrt(1 - EARTH_E2 * s * s)                  # prime vertical
    north = np.radians(np.asarray(lat) - lat0) * r_n
    east = np.radians(np.asarray(lon) - lon0) * r_e * math.cos(phi)
    down = -(np.asarray(alt) - alt0)
    return np.stack([north, east, down], axis=-1)


def read_messages(path):
    """One pass over the log, collecting the raw rows we need"""
    log = mavutil.mavlink_connection(path)
    rows = {k: [] for k in ('IMU', 'GPS', 'BARO', 'MAG', 'SIM', 'SIM2', 'XKF1', 'POS')}
    origin = None
    gps_lag_ms = None
    mag_dec = 0.0
    while True:
        msg = log.recv_match(type=list(rows) + ['ORGN', 'PARM'])
        if msg is None:
            break
        kind = msg.get_type()
        if kind == 'ORGN':
            if msg.Type == 0 and origin is None:
                origin = (msg.Lat, msg.Lng, msg.Alt)
        elif kind == 'PARM':
            if msg.Name == 'SIM_GPS1_LAG_MS':
                gps_lag_ms = msg.Value
            elif msg.Name == 'COMPASS_DEC':
                mag_dec = msg.Value
        elif kind in ('IMU', 'BARO', 'GPS', 'MAG'):
            if msg.I == 0:
                rows[kind].append(msg)
        elif kind == 'XKF1':
            if msg.C == 0:
                rows[kind].append(msg)
        else:
            rows[kind].append(msg)
    if origin is None:
        raise RuntimeError('No EKF origin (ORGN) in log - was the vehicle ever armed?')
    return rows, origin, gps_lag_ms, mag_dec


def load_log(path):
    rows, origin, gps_lag_ms, mag_dec = read_messages(path)

    def t(msgs):
        return np.array([m.TimeUS / 1e6 for m in msgs])

    data = {'origin': np.array(origin),
            'gps_lag_s': np.array((gps_lag_ms or 0.0) / 1000.0)}

    imu = rows['IMU']
    data['imu_t'] = t(imu)
    data['imu_gyro'] = np.array([[m.GyrX, m.GyrY, m.GyrZ] for m in imu])
    data['imu_acc'] = np.array([[m.AccX, m.AccY, m.AccZ] for m in imu])

    gps = [m for m in rows['GPS'] if m.Status >= GPS_FIX_3D]
    data['gps_t'] = t(gps)
    data['gps_pos'] = lla_to_ned([m.Lat for m in gps], [m.Lng for m in gps],
                                 [m.Alt for m in gps], origin)
    # Logged as ground speed + course over ground, and VZ already NED (down +)
    course = np.radians([m.GCrs for m in gps])
    speed = np.array([m.Spd for m in gps])
    data['gps_vel'] = np.stack([speed * np.cos(course), speed * np.sin(course),
                                np.array([m.VZ for m in gps])], axis=-1)
    data['gps_nsats'] = np.array([m.NSats for m in gps])
    data['gps_hdop'] = np.array([m.HDop for m in gps])

    baro = rows['BARO']
    data['baro_t'] = t(baro)
    data['baro_alt'] = np.array([m.Alt for m in baro])

    mag = rows['MAG']
    data['mag_t'] = t(mag)
    data['mag_field'] = np.array([[m.MagX, m.MagY, m.MagZ] for m in mag])
    data['mag_dec'] = np.array(mag_dec)

    # SIM2 is 400 Hz but relative to SITL's own home, not the EKF origin.
    # SIM (10 Hz) has lat/lon/alt, so use it to find the constant offset.
    sim2, sim = rows['SIM2'], rows['SIM']
    truth_t = t(sim2)
    truth_pos_home = np.array([[m.PN, m.PE, m.PD] for m in sim2])
    sim_t = t(sim)
    sim_ned = lla_to_ned([m.Lat for m in sim], [m.Lng for m in sim],
                         [m.Alt for m in sim], origin)
    idx = np.clip(np.searchsorted(truth_t, sim_t), 0, len(truth_t) - 1)
    offset = np.median(sim_ned - truth_pos_home[idx], axis=0)
    data['truth_t'] = truth_t
    data['truth_pos'] = truth_pos_home + offset
    data['truth_vel'] = np.array([[m.VN, m.VE, m.VD] for m in sim2])
    data['truth_att_t'] = sim_t
    data['truth_quat'] = np.array([[m.Q1, m.Q2, m.Q3, m.Q4] for m in sim])

    ekf = rows['XKF1']
    data['ekf_t'] = t(ekf)
    data['ekf_pos'] = np.array([[m.PN, m.PE, m.PD] for m in ekf])
    data['ekf_vel'] = np.array([[m.VN, m.VE, m.VD] for m in ekf])
    data['ekf_att'] = np.array([[m.Roll, m.Pitch, m.Yaw] for m in ekf])

    used = rows['POS']
    data['used_t'] = t(used)
    data['used_pos'] = lla_to_ned([m.Lat for m in used], [m.Lng for m in used],
                                  [m.Alt for m in used], origin)

    return data


def at(t_query, t_src, values):
    """Sample values (rows matching t_src) at t_query by linear interpolation"""
    return np.stack([np.interp(t_query, t_src, values[:, i])
                     for i in range(values.shape[1])], axis=-1)


def summary(data):
    """Print rates and sanity checks against truth"""
    def rate(key):
        tt = data[key]
        return (len(tt) - 1) / (tt[-1] - tt[0])

    print(f'origin            {data["origin"][0]:.7f}, {data["origin"][1]:.7f}, '
          f'{data["origin"][2]:.2f} m')
    for name, key in (('IMU', 'imu_t'), ('GPS (3D fix)', 'gps_t'), ('baro', 'baro_t'),
                      ('truth pos/vel', 'truth_t'), ('truth attitude', 'truth_att_t'),
                      ('EKF3', 'ekf_t')):
        print(f'{name:17} {len(data[key]):6} samples, {rate(key):6.1f} Hz')
    dt = np.diff(data['imu_t'])
    print(f'IMU dt            mean {dt.mean() * 1e3:.2f} ms, max {dt.max() * 1e3:.2f} ms')

    # GPS against truth, now and at the configured lag
    lag = float(data['gps_lag_s'])
    for label, shift in (('GPS vs truth', 0.0), (f'... {lag * 1e3:.0f} ms earlier', lag)):
        truth = at(data['gps_t'] - shift, data['truth_t'], data['truth_pos'])
        err = np.linalg.norm(data['gps_pos'] - truth, axis=1)
        print(f'{label:17} position error mean {err.mean():.3f} m, max {err.max():.3f} m')

    truth = at(data['ekf_t'], data['truth_t'], data['truth_pos'])
    err = np.linalg.norm(data['ekf_pos'][:, :2] - truth[:, :2], axis=1)
    print(f'EKF3 vs truth     horizontal error mean {err.mean():.3f} m, max {err.max():.3f} m')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('log')
    parser.add_argument('--out', help='npz path (default: next to the log, .npz)')
    args = parser.parse_args()

    data = load_log(args.log)
    summary(data)

    out = args.out or os.path.splitext(args.log)[0] + '.npz'
    np.savez_compressed(out, **data)
    print(f'saved {out}')


if __name__ == '__main__':
    main()
