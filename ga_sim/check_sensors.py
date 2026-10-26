#!/usr/bin/env python3
"""
Sensor check for the rangefinder world: take off, hover, compare the
rangefinder against altitude, confirm optical flow is reporting, land.

Usage: python3 ga_sim/check_sensors.py [--connect URL] [--alt M] [--hover S]

Run against SITL started by launch_sim.sh. Default connection is SITL's
SERIAL1 TCP port, which is free whether or not MAVProxy is running.
"""

import argparse
import statistics
import time

from pymavlink import mavutil


SENSOR_MSGS = {
    'DISTANCE_SENSOR': 132,
    'RANGEFINDER': 173,
    'OPTICAL_FLOW': 100,
    'LOCAL_POSITION_NED': 32,
}


def request_streams(mav, rate_hz):
    for msg_id in SENSOR_MSGS.values():
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / rate_hz), 0, 0, 0, 0, 0)


def get_param(mav, name, timeout=5.0):
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component, name.encode(), -1)
    msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout)
    while msg is not None and msg.param_id != name:
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout)
    return None if msg is None else msg.param_value


def arm_guided(mav, timeout=180.0):
    """Retry GUIDED + arm until pre-arm checks pass (EKF needs time to settle)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        mav.set_mode('GUIDED')
        mav.arducopter_arm()
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return True
        text = mav.recv_match(type='STATUSTEXT', blocking=False)
        if text:
            print(f'   FC: {text.text}')
        time.sleep(2)
    return False


def sample(mav, seconds):
    """Collect (alt, range) pairs and flow messages for a while."""
    alt = None
    pairs, flow = [], []
    end = time.time() + seconds
    while time.time() < end:
        msg = mav.recv_match(type=list(SENSOR_MSGS), blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == 'LOCAL_POSITION_NED':
            alt = -msg.z
        elif kind == 'DISTANCE_SENSOR' and alt is not None:
            pairs.append((alt, msg.current_distance / 100.0))
        elif kind == 'OPTICAL_FLOW':
            flow.append(msg)
    return pairs, flow


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--connect', default='tcp:127.0.0.1:5762')
    parser.add_argument('--alt', type=float, default=3.0)
    parser.add_argument('--hover', type=float, default=10.0)
    args = parser.parse_args()

    print(f'🔌 Connecting to {args.connect}')
    mav = mavutil.mavlink_connection(args.connect, source_system=250)
    mav.wait_heartbeat()
    print(f'✅ Heartbeat from system {mav.target_system}')

    for name in ('RNGFND1_TYPE', 'RNGFND1_ORIENT', 'FLOW_TYPE', 'SIM_FLOW_ENABLE'):
        print(f'   {name} = {get_param(mav, name)}')

    request_streams(mav, rate_hz=5)

    print('⏳ Waiting for pre-arm checks, then arming in GUIDED')
    if not arm_guided(mav):
        print('❌ Could not arm')
        return 1

    print(f'🚀 Takeoff to {args.alt} m')
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, args.alt)
    sample(mav, 12)  # climb

    print(f'📏 Hovering {args.hover} s')
    pairs, flow = sample(mav, args.hover)

    print('🛬 Landing')
    mav.set_mode('LAND')

    if pairs:
        alts = [a for a, _ in pairs]
        ranges = [r for _, r in pairs]
        diffs = [r - a for a, r in pairs]
        print(f'\nRangefinder ({len(pairs)} samples):')
        print(f'   altitude  mean {statistics.mean(alts):.2f} m')
        print(f'   range     mean {statistics.mean(ranges):.2f} m')
        print(f'   range-alt mean {statistics.mean(diffs):+.2f} m, '
              f'stdev {statistics.pstdev(diffs):.3f} m')
    else:
        print('\n❌ No DISTANCE_SENSOR messages received')

    if flow:
        quality = [m.quality for m in flow]
        print(f'Optical flow ({len(flow)} samples): quality mean '
              f'{statistics.mean(quality):.0f}, ground_distance last '
              f'{flow[-1].ground_distance:.2f} m')
    else:
        print('❌ No OPTICAL_FLOW messages received')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
