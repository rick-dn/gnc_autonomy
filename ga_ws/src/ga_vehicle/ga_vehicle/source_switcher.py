#!/usr/bin/env python3
"""
Source Switcher - off-board GPS -> optical flow fallback.
Watches GPS health over MAVROS and selects the EKF3 source set:
set 1 (GPS) while GPS is healthy, set 2 (optical flow) when it is not.
Source sets are defined in ga_sim/params/p1_sources.parm.
"""

import rclpy
from rclpy.node import Node

from ga_vehicle.mavros_adapter import MavrosAdapter


# GPS_FIX_TYPE_3D_FIX - anything below is treated as no usable GPS
MIN_FIX_TYPE = 3
MIN_SATELLITES = 6

# No GPSRAW for this long counts as GPS lost, whatever the last values were
GPS_STALE_S = 1.0

# Hysteresis: GPS must be bad this long before falling back to flow, and good
# this long before returning to GPS - avoids flapping on a marginal fix.
BAD_HOLD_S = 1.0
GOOD_HOLD_S = 5.0

GPS_SET = 1
FLOW_SET = 2

CHECK_RATE_HZ = 10.0


class SourceSwitcher(Node):
    """Selects the EKF source set from GPS health"""

    def __init__(self):
        super().__init__('source_switcher')
        self.mavros = MavrosAdapter(self)

        # ArduPilot boots on set 1, so start there without commanding it
        self.active_set = GPS_SET

        # Time the current GPS health state began; reset on every change
        self.gps_ok = None
        self.state_since = self.get_clock().now()

        self.timer = self.create_timer(1.0 / CHECK_RATE_HZ, self.check)
        self.get_logger().info(
            f'Source switcher running: fix>={MIN_FIX_TYPE}, sats>={MIN_SATELLITES}, '
            f'bad {BAD_HOLD_S}s -> set {FLOW_SET}, good {GOOD_HOLD_S}s -> set {GPS_SET}'
        )

    def gps_healthy(self, now):
        """True if the latest GPS status is current and good enough to fly on"""
        if self.mavros.gps_raw_time is None:
            return False
        age = (now - self.mavros.gps_raw_time).nanoseconds / 1e9
        if age > GPS_STALE_S:
            return False
        return (self.mavros.gps_fix_type >= MIN_FIX_TYPE and
                self.mavros.gps_satellites_visible >= MIN_SATELLITES)

    def check(self):
        now = self.get_clock().now()
        ok = self.gps_healthy(now)

        if ok != self.gps_ok:
            self.gps_ok = ok
            self.state_since = now
            self.get_logger().info(
                f'GPS {"healthy" if ok else "BAD"}: fix={self.mavros.gps_fix_type}, '
                f'sats={self.mavros.gps_satellites_visible}'
            )

        held = (now - self.state_since).nanoseconds / 1e9

        if not ok and self.active_set == GPS_SET and held >= BAD_HOLD_S:
            self.get_logger().warn(f'GPS bad for {held:.1f}s - switching to optical flow')
            if self.mavros.set_ekf_source_set(FLOW_SET):
                self.active_set = FLOW_SET

        elif ok and self.active_set == FLOW_SET and held >= GOOD_HOLD_S:
            self.get_logger().info(f'GPS good for {held:.1f}s - switching back to GPS')
            if self.mavros.set_ekf_source_set(GPS_SET):
                self.active_set = GPS_SET


def main(args=None):
    rclpy.init(args=args)
    node = SourceSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
