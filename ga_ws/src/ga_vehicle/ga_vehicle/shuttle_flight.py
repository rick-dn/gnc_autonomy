#!/usr/bin/env python3
"""
Shuttle Flight - scripted out-and-back flight in local ENU coordinates.
Takes off to 5 m, then flies along the takeoff heading:
  +5 m, turn, back 7 m, turn, forward 7 m, turn, back 5 m (home), land.
Each turn is a 180 deg yaw in place, so every leg is flown nose-first.

Used to record the custom-estimator flight (scripts/start_estimator.sh), so
the same path can be repeated. Run with SITL + MAVROS up, vehicle disarmed.
"""

import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ga_vehicle.mavros_adapter import MavrosAdapter


TAKEOFF_ALT_M = 5.0

# Leg end points, metres along the takeoff heading from the takeoff point
LEG_POSITIONS_M = [5.0, -2.0, 5.0, 0.0]

# Arrival: within this distance and below this speed, held for SETTLE_S
POS_TOL_M = 0.3
SPEED_TOL_MS = 0.3
YAW_TOL_DEG = 5.0
SETTLE_S = 1.0

LEG_TIMEOUT_S = 30.0
ARM_TIMEOUT_S = 120.0


class ShuttleFlight(Node):
    """Flies the shuttle path through the MAVROS adapter"""

    def __init__(self):
        super().__init__('shuttle_flight')
        self.mavros = MavrosAdapter(self)

    def log(self, text):
        self.get_logger().info(text)

    def telemetry(self):
        return self.mavros.get_telemetry()

    def wait_for(self, condition, timeout, what):
        """Poll condition() until it has held for SETTLE_S, or time out"""
        deadline = time.time() + timeout
        held_since = None
        while time.time() < deadline:
            if condition():
                held_since = held_since or time.time()
                if time.time() - held_since >= SETTLE_S:
                    return True
            else:
                held_since = None
            time.sleep(0.1)
        self.get_logger().error(f'Timed out waiting for {what}')
        return False

    def arm_and_takeoff(self):
        """GUIDED, arm (retrying until pre-arm checks pass), climb to TAKEOFF_ALT_M"""
        deadline = time.time() + ARM_TIMEOUT_S
        while not self.mavros.armed and time.time() < deadline:
            self.mavros.set_mode_guided()
            self.mavros.arm()
            time.sleep(3.0)
        if not self.mavros.armed:
            self.get_logger().error('Could not arm - pre-arm checks failing?')
            return False

        self.mavros.takeoff(TAKEOFF_ALT_M)
        return self.wait_for(
            lambda: abs(self.telemetry()['local_position']['up'] - TAKEOFF_ALT_M) < POS_TOL_M,
            LEG_TIMEOUT_S, 'takeoff altitude')

    def turn_to(self, east, north, yaw_deg):
        """Yaw in place at (east, north)"""
        self.mavros.goto_local(east, north, TAKEOFF_ALT_M, yaw_deg)

        def yaw_done():
            error = (self.telemetry()['local_position']['yaw_deg'] - yaw_deg + 180.0) % 360.0 - 180.0
            return abs(error) < YAW_TOL_DEG

        return self.wait_for(yaw_done, LEG_TIMEOUT_S, f'yaw {yaw_deg:.0f} deg')

    def fly_to(self, east, north, yaw_deg):
        """Fly to (east, north) at TAKEOFF_ALT_M, holding yaw"""
        self.mavros.goto_local(east, north, TAKEOFF_ALT_M, yaw_deg)

        def arrived():
            t = self.telemetry()
            pos = t['local_position']
            vel = t['velocity_body']
            dist = math.dist((pos['east'], pos['north'], pos['up']),
                             (east, north, TAKEOFF_ALT_M))
            speed = math.hypot(vel['forward'], vel['right'])
            return dist < POS_TOL_M and speed < SPEED_TOL_MS

        return self.wait_for(arrived, LEG_TIMEOUT_S, f'E={east:.1f} N={north:.1f}')

    def run(self):
        if not self.arm_and_takeoff():
            return False

        # Path is defined along the heading at takeoff, from where it took off
        start = self.telemetry()['local_position']
        heading = start['yaw_deg']
        dir_e = math.cos(math.radians(heading))
        dir_n = math.sin(math.radians(heading))
        self.log(f'Takeoff done at E={start["east"]:.1f} N={start["north"]:.1f}, '
                 f'heading {heading:.0f} deg')

        along = 0.0
        yaw = heading
        for i, target in enumerate(LEG_POSITIONS_M):
            here_e = start['east'] + along * dir_e
            here_n = start['north'] + along * dir_n

            # Face the direction of travel - a 180 deg turn on every leg after the first
            leg_yaw = heading if target > along else (heading + 180.0) % 360.0
            if i > 0 and leg_yaw != yaw:
                self.log(f'Turning around to {leg_yaw:.0f} deg')
                if not self.turn_to(here_e, here_n, leg_yaw):
                    return False
            yaw = leg_yaw

            east = start['east'] + target * dir_e
            north = start['north'] + target * dir_n
            self.log(f'Leg {i + 1}: {abs(target - along):.0f} m to {target:+.0f} m along heading')
            if not self.fly_to(east, north, yaw):
                return False
            along = target

        return True


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleFlight()

    # Callbacks run on a background executor; the flight logic polls the
    # adapter's state from the main thread
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Let the first state/odometry messages arrive
        time.sleep(2.0)
        ok = node.run()
        node.log('Path complete - landing' if ok else 'Aborting - landing')
        node.mavros.set_mode('LAND')
        node.wait_for(lambda: not node.mavros.armed, 60.0, 'disarm')
        node.log('Landed and disarmed')
    except KeyboardInterrupt:
        node.log('Interrupted - landing')
        node.mavros.set_mode('LAND')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
