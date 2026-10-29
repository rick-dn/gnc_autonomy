#!/usr/bin/env python3
"""
Batch fault runner - fresh ArduPilot SITL per run, scripted flight, fault at a
fixed time, one dataflash log per run.

Usage:
  python3 -m ga_faults.run --list
  python3 -m ga_faults.run --fault gps_loss --runs 5 [--speedup 5]
  python3 -m ga_faults.run --fault all --runs 3 [--no-noise]

Each run starts arducopter on its built-in physics (no Gazebo, no ROS) in
runs/faults/<fault>/run_<k>/, connects over pymavlink, and:
  1. arms in GUIDED and takes off to TAKEOFF_ALT_M
  2. flies the shuttle along the home heading (+5, -2, +5, 0 m) on repeat
  3. injects the fault FAULT_AT_S + 0..FAULT_JITTER_S after takeoff (seeded by
     run number), clears it FAULT_FOR_S later
  4. flies AFTER_S more, then lands
The fault parameters' original values are read first and restored on clear.
All timing uses the vehicle clock (time_boot_ms), so it holds at any speedup.
run.json records the fault and the exact inject/clear times.
"""

import argparse
import json
import math
import random
import shutil
import subprocess
import time
from pathlib import Path

from pymavlink import mavutil

from ga_faults.faults import FAULTS


ROOT = Path(__file__).resolve().parents[1]
ARDUPILOT = Path('/data/projects/udl_aakash/tools/ardupilot')
BINARY = ARDUPILOT / 'build/sitl/bin/arducopter'
PARAM_FILES = [ARDUPILOT / 'Tools/autotest/default_params/copter.parm',
               ROOT / 'ga_sim/params/batch.parm',      # sensors, Lua switching, logging
               ROOT / 'ga_sim/params/p1_sources.parm']  # EKF3 source sets
NOISE_PARAM_FILE = ROOT / 'ga_sim/params/batch_noise.parm'
RUNS_DIR = ROOT / 'runs/faults'

# Lua scripts copied into each run's scripts/ folder, where SITL loads them from
SCRIPTS = [ROOT / 'ga_sim/scripts/ahrs-source-gps-optflow-auto.lua']

# Same spot as the Gazebo runs; heading 0 so "forward" is north (+x in NED)
HOME = '51.2805029,-1.1035530,100.19,0'

TAKEOFF_ALT_M = 5.0
LEG_POSITIONS_M = [5.0, -2.0, 5.0, 0.0]

# Timeline, sim seconds after takeoff completes. The fault goes in at
# FAULT_AT_S plus a random 0..FAULT_JITTER_S (seeded by run number), so repeat
# runs catch the shuttle at different points - SITL's sensor noise is the same
# sequence every run, so this is the main source of run-to-run spread.
FAULT_AT_S = 30.0
FAULT_JITTER_S = 10.0
FAULT_FOR_S = 60.0
AFTER_S = 20.0

POS_TOL_M = 0.3
SPEED_TOL_MS = 0.3
YAW_TOL_DEG = 5.0
LEG_TIMEOUT_S = 25.0          # a leg that doesn't arrive (e.g. EKF off) just moves on
ARM_TIMEOUT_S = 180.0
LAND_TIMEOUT_S = 60.0

STREAMS_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2,
}

# Position + yaw only: ignore velocity, acceleration and yaw rate
POS_YAW_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)


class FaultRun:
    """One SITL instance, one flight, one fault"""

    def __init__(self, fault_name, run_dir, speedup, instance, run_index, noise):
        self.fault_name = fault_name
        self.fault = FAULTS[fault_name]
        self.run_dir = run_dir
        self.speedup = speedup
        self.param_files = PARAM_FILES[:2] + ([NOISE_PARAM_FILE] if noise else []) + PARAM_FILES[2:]
        self.fault_at_s = FAULT_AT_S + random.Random(run_index).uniform(0.0, FAULT_JITTER_S)
        self.port = 5760 + 10 * instance
        self.instance = instance

        self.sitl = None
        self.mav = None

        # Vehicle state, updated by pump()
        self.boot_s = 0.0
        self.armed = False
        self.pos = (0.0, 0.0, 0.0)       # NED, EKF estimate
        self.vel = (0.0, 0.0, 0.0)
        self.yaw = 0.0                   # rad, NED

        # Scheduled param sets: [(sim_time, label, {name: value})]
        self.schedule = []
        self.record = {'fault': fault_name, 'description': self.fault['description'],
                       'set': self.fault['set'], 'speedup': speedup, 'home': HOME,
                       'noise': noise, 'fault_after_takeoff_s': round(self.fault_at_s, 2),
                       'events': {}, 'statustext': [], 'completed': False}

    # --- SITL process and link ------------------------------------------------

    def start(self):
        if self.run_dir.exists():
            shutil.rmtree(self.run_dir)
        self.run_dir.mkdir(parents=True)
        (self.run_dir / 'scripts').mkdir()
        for script in SCRIPTS:
            shutil.copy(script, self.run_dir / 'scripts')
        cmd = [str(BINARY), '-w', '--model', '+', '--speedup', str(self.speedup),
               '--home', HOME, '-I', str(self.instance),
               '--defaults', ','.join(str(p) for p in self.param_files)]
        self.sitl_log = open(self.run_dir / 'sitl.log', 'w')
        self.sitl = subprocess.Popen(cmd, cwd=self.run_dir, stdout=self.sitl_log,
                                     stderr=subprocess.STDOUT)

        deadline = time.time() + 30
        while True:
            try:
                self.mav = mavutil.mavlink_connection(f'tcp:127.0.0.1:{self.port}',
                                                      source_system=255)
                break
            except (ConnectionRefusedError, OSError):
                if time.time() > deadline or self.sitl.poll() is not None:
                    raise RuntimeError(f'SITL did not open port {self.port}, '
                                       f'see {self.run_dir / "sitl.log"}')
                time.sleep(0.5)
        self.mav.wait_heartbeat(timeout=30)
        for msg_id, hz in STREAMS_HZ.items():
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def stop(self):
        if self.mav:
            self.mav.close()
        if self.sitl and self.sitl.poll() is None:
            self.sitl.terminate()
            try:
                self.sitl.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.sitl.kill()
        if self.sitl:
            self.sitl_log.close()

    def command(self, cmd, *params):
        p = list(params) + [0] * (7 - len(params))
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       cmd, 0, *p)

    # --- message pump -----------------------------------------------------------

    def pump(self, seconds=0.1):
        """Read messages for up to `seconds` of wall time, run due schedule items"""
        end = time.time() + seconds
        while time.time() < end:
            msg = self.mav.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            kind = msg.get_type()
            if hasattr(msg, 'time_boot_ms'):
                self.boot_s = max(self.boot_s, msg.time_boot_ms / 1000.0)
            if kind == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == 'LOCAL_POSITION_NED':
                self.pos = (msg.x, msg.y, msg.z)
                self.vel = (msg.vx, msg.vy, msg.vz)
            elif kind == 'ATTITUDE':
                self.yaw = msg.yaw
            elif kind == 'STATUSTEXT':
                self.record['statustext'].append([round(self.boot_s, 2), msg.text])
                print(f'      [{self.boot_s:7.1f}] FC: {msg.text}')
            elif kind == 'PARAM_VALUE':
                self.param_ack(msg)
        self.run_schedule()

    def run_schedule(self):
        due = [s for s in self.schedule if s[0] <= self.boot_s]
        for item in due:
            self.schedule.remove(item)
            _, label, params = item
            for name, value in params.items():
                self.mav.param_set_send(name, float(value))
            self.record['events'][label] = {'sent_s': round(self.boot_s, 3), 'acked_s': None}
            print(f'    {label} at {self.boot_s:.1f} s: {params}')

    def param_ack(self, msg):
        """Mark a scheduled event acknowledged when the FC echoes the new value"""
        for label, event in self.record['events'].items():
            if event['acked_s'] is not None:
                continue
            params = self.fault['set'] if label == 'inject' else self.record['original']
            if msg.param_id in params and math.isclose(msg.param_value, params[msg.param_id],
                                                       rel_tol=1e-4, abs_tol=1e-9):
                event['acked_s'] = round(self.boot_s, 3)

    def wait_sim(self, condition, timeout_s, what=None):
        """Pump until condition() holds for 1 s of sim time, or timeout (sim seconds)"""
        start = self.boot_s
        held_since = None
        while self.boot_s - start < timeout_s:
            self.pump()
            if condition():
                held_since = held_since if held_since is not None else self.boot_s
                if self.boot_s - held_since >= 1.0:
                    return True
            else:
                held_since = None
            if self.sitl.poll() is not None:
                raise RuntimeError('SITL exited')
        if what:
            print(f'    timed out waiting for {what}')
        return False

    def get_param(self, name):
        self.mav.param_fetch_one(name)
        deadline = time.time() + 5
        while time.time() < deadline:
            msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg and msg.param_id == name:
                return msg.param_value
        raise RuntimeError(f'could not read {name}')

    # --- flight -------------------------------------------------------------------

    def arm_and_takeoff(self):
        start = self.boot_s
        while not self.armed:
            if self.boot_s - start > ARM_TIMEOUT_S:
                raise RuntimeError('could not arm - pre-arm checks failing?')
            self.mav.set_mode('GUIDED')
            self.mav.arducopter_arm()
            self.pump(1.0)
            self.wait_sim(lambda: self.armed, 3.0)
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M)
        if not self.wait_sim(lambda: abs(-self.pos[2] - TAKEOFF_ALT_M) < POS_TOL_M,
                             LEG_TIMEOUT_S, 'takeoff altitude'):
            raise RuntimeError('takeoff did not reach altitude')

    def goto(self, north, east, yaw):
        self.mav.mav.set_position_target_local_ned_send(
            int(self.boot_s * 1000), self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, POS_YAW_MASK,
            north, east, -TAKEOFF_ALT_M, 0, 0, 0, 0, 0, 0, yaw, 0)

    def turn_to(self, north, east, yaw):
        self.goto(north, east, yaw)
        # A failsafe may land and disarm mid-turn - stop waiting if so
        self.wait_sim(lambda: not self.armed or abs(math.degrees(
            (self.yaw - yaw + math.pi) % (2 * math.pi) - math.pi)) < YAW_TOL_DEG,
            LEG_TIMEOUT_S, 'turn')

    def fly_to(self, north, east, yaw):
        self.goto(north, east, yaw)

        def arrived():
            if not self.armed:
                return True
            dist = math.dist(self.pos, (north, east, -TAKEOFF_ALT_M))
            return dist < POS_TOL_M and math.hypot(self.vel[0], self.vel[1]) < SPEED_TOL_MS

        self.wait_sim(arrived, LEG_TIMEOUT_S, f'N={north:.1f} E={east:.1f}')

    def fly_shuttle_until(self, end_s, start_n, start_e, heading):
        """Repeat the shuttle legs until sim time end_s, or until the vehicle
        disarms on its own (e.g. a failsafe landed it)"""
        dir_n, dir_e = math.cos(heading), math.sin(heading)
        along = 0.0
        yaw = heading
        while self.boot_s < end_s:
            for target in LEG_POSITIONS_M:
                if not self.armed:
                    self.record['events']['disarmed_early'] = {
                        'sent_s': round(self.boot_s, 3), 'acked_s': round(self.boot_s, 3)}
                    print(f'    vehicle disarmed on its own at {self.boot_s:.1f} s')
                    return
                if self.boot_s >= end_s:
                    return
                leg_yaw = heading if target > along else heading + math.pi
                if leg_yaw != yaw:
                    self.turn_to(start_n + along * dir_n, start_e + along * dir_e, leg_yaw)
                    yaw = leg_yaw
                self.fly_to(start_n + target * dir_n, start_e + target * dir_e, yaw)
                along = target

    def land(self):
        self.mav.set_mode('LAND')
        self.wait_sim(lambda: not self.armed, LAND_TIMEOUT_S, 'disarm')

    def fly(self):
        self.record['original'] = {name: self.get_param(name) for name in self.fault['set']}
        self.arm_and_takeoff()
        takeoff_s = self.boot_s
        self.record['events']['takeoff'] = {'sent_s': round(takeoff_s, 3),
                                            'acked_s': round(takeoff_s, 3)}
        print(f'    airborne at {takeoff_s:.1f} s')

        inject_s = takeoff_s + self.fault_at_s
        clear_s = inject_s + FAULT_FOR_S
        end_s = clear_s + AFTER_S
        if self.fault['set']:
            self.schedule.append((inject_s, 'inject', self.fault['set']))
            self.schedule.append((clear_s, 'clear', self.record['original']))

        self.fly_shuttle_until(end_s, self.pos[0], self.pos[1], self.yaw)
        if 'disarmed_early' in self.record['events']:
            # Landed by a failsafe - stay on the ground until the fault has
            # been cleared, so the log still shows the EKF recovering
            self.wait_sim(lambda: False, max(clear_s + 5.0 - self.boot_s, 0.0))
        else:
            print(f'    landing at {self.boot_s:.1f} s')
            self.land()
        self.record['completed'] = True


def run_one(fault_name, k, speedup, instance, noise):
    run_dir = RUNS_DIR / fault_name / f'run_{k}'
    print(f'  {fault_name} run {k} -> {run_dir.relative_to(ROOT)}')
    run = FaultRun(fault_name, run_dir, speedup, instance, k, noise)
    started = time.time()
    try:
        run.start()
        run.fly()
    except Exception as exc:
        run.record['error'] = str(exc)
        print(f'    FAILED: {exc}')
    finally:
        run.stop()
        run.record['wall_s'] = round(time.time() - started, 1)
        logs = sorted((run_dir / 'logs').glob('*.BIN'))
        run.record['log'] = str(logs[-1].relative_to(run_dir)) if logs else None
        (run_dir / 'run.json').write_text(json.dumps(run.record, indent=2))
    print(f'    done in {run.record["wall_s"]} s wall, '
          f'{"completed" if run.record["completed"] else "NOT completed"}')
    return run.record


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--fault', help="fault name, comma list, or 'all'")
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--speedup', type=int, default=5)
    parser.add_argument('--instance', type=int, default=0,
                        help='SITL instance (ports 5760 + 10*N)')
    parser.add_argument('--no-noise', action='store_true',
                        help='leave out batch_noise.parm (clean sensors)')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()

    if args.list or not args.fault:
        for name, fault in FAULTS.items():
            print(f'{name:15} {fault["description"]}')
        return

    names = list(FAULTS) if args.fault == 'all' else args.fault.split(',')
    for name in names:
        if name not in FAULTS:
            raise SystemExit(f'unknown fault {name!r} - see --list')
    for name in names:
        for k in range(1, args.runs + 1):
            run_one(name, k, args.speedup, args.instance, not args.no_noise)


if __name__ == '__main__':
    main()
