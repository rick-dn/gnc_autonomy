"""
Error-state EKF for INS/GNSS - IMU propagation, GPS and baro corrections.

Frames: navigation is local NED, body is FRD. The attitude quaternion q is
Hamilton [w, x, y, z] rotating body -> NED, so v_ned = R(q) @ v_body.

Nominal state (integrated directly from the IMU):
  p  position NED (m)          v  velocity NED (m/s)       q  attitude
  ba accelerometer bias (m/s^2)                            bg gyro bias (rad/s)

Error state (what the Kalman filter estimates, 15 x 1):
  dp(3) dv(3) dtheta(3) dba(3) dbg(3)
where dtheta is a small rotation in the body frame: q_true = q * exp(dtheta).

Follows the structure in Sola, "Quaternion kinematics for the error-state
Kalman filter" (2017), sections 5 and 6.
"""

import numpy as np


GRAVITY = 9.80665
G_NED = np.array([0.0, 0.0, GRAVITY])

# Slices into the 15-element error state
P_ = slice(0, 3)
V_ = slice(3, 6)
TH = slice(6, 9)
BA = slice(9, 12)
BG = slice(12, 15)


# --- quaternion helpers -----------------------------------------------------

def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def quat_from_rotvec(r):
    """Quaternion for a rotation vector r (axis * angle, rad)"""
    angle = np.linalg.norm(r)
    if angle < 1e-12:
        q = np.array([1.0, *(0.5 * r)])
        return q / np.linalg.norm(q)
    axis = r / angle
    return np.array([np.cos(angle / 2), *(np.sin(angle / 2) * axis)])


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_from_euler(roll, pitch, yaw):
    """ZYX Euler angles (rad) -> body->NED quaternion"""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


def quat_to_euler(q):
    """Body->NED quaternion -> roll, pitch, yaw (rad), ZYX"""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


# --- filter -----------------------------------------------------------------

class ErrorStateEKF:
    """15-state error-state EKF: predict on IMU, correct on any linear(ised) measurement"""

    def __init__(self, p, v, q, noise, init_std):
        """
        noise: continuous-time densities
          acc (m/s^2/sqrt(Hz)), gyro (rad/s/sqrt(Hz)),
          acc_bias (m/s^3/sqrt(Hz)), gyro_bias (rad/s^2/sqrt(Hz))
        init_std: 1-sigma of the initial error, keys pos, vel, att, acc_bias, gyro_bias
        """
        self.p = np.array(p, dtype=float)
        self.v = np.array(v, dtype=float)
        self.q = np.array(q, dtype=float)
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)
        self.noise = noise

        std = np.concatenate([np.full(3, init_std['pos']), np.full(3, init_std['vel']),
                              np.asarray(init_std['att'], dtype=float),
                              np.full(3, init_std['acc_bias']),
                              np.full(3, init_std['gyro_bias'])])
        self.P = np.diag(std ** 2)

    def predict(self, gyro, acc, dt):
        """Propagate the nominal state and the error covariance by one IMU sample"""
        w = gyro - self.bg
        f = acc - self.ba          # specific force, body
        R = quat_to_rot(self.q)
        a = R @ f + G_NED          # acceleration, NED

        self.p = self.p + self.v * dt + 0.5 * a * dt * dt
        self.v = self.v + a * dt
        dq = quat_from_rotvec(w * dt)
        self.q = quat_mul(self.q, dq)
        self.q /= np.linalg.norm(self.q)

        # Error-state transition, first order in dt (Sola eq. 269)
        F = np.eye(15)
        F[P_, V_] = np.eye(3) * dt
        F[V_, TH] = -R @ skew(f) * dt
        F[V_, BA] = -R * dt
        F[TH, TH] = quat_to_rot(dq).T
        F[TH, BG] = -np.eye(3) * dt

        n = self.noise
        Q = np.zeros(15)
        Q[V_] = n['acc'] ** 2 * dt
        Q[TH] = n['gyro'] ** 2 * dt
        Q[BA] = n['acc_bias'] ** 2 * dt
        Q[BG] = n['gyro_bias'] ** 2 * dt

        self.P = F @ self.P @ F.T + np.diag(Q)

    def nis(self, residual, H, R_meas):
        """
        Normalised innovation squared, residual' S^-1 residual. Chi-square
        distributed with len(residual) degrees of freedom if the filter and
        the measurement agree - large values mean they don't.
        """
        S = H @ self.P @ H.T + R_meas
        return float(residual @ np.linalg.solve(S, residual))

    def reset_position(self, p, pos_var):
        """Jump position onto a measurement and reset its uncertainty"""
        self.p = np.array(p, dtype=float)
        self.P[P_, :] = 0.0
        self.P[:, P_] = 0.0
        self.P[P_, P_] = np.diag(pos_var)

    def correct(self, residual, H, R_meas, frozen=None):
        """
        Kalman update on the error state, then fold it into the nominal state.
        residual = measurement - prediction; H maps the error state to it.
        frozen: error-state indices this measurement must not change (their
        gain rows are zeroed, a "consider" update).
        Returns the innovation covariance S (for checking the residual size).
        """
        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)
        if frozen is not None:
            K[frozen, :] = 0.0
        dx = K @ residual

        # Joseph form keeps P symmetric and positive definite
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T

        self.p += dx[P_]
        self.v += dx[V_]
        self.q = quat_mul(self.q, quat_from_rotvec(dx[TH]))
        self.q /= np.linalg.norm(self.q)
        self.ba += dx[BA]
        self.bg += dx[BG]
        return S
