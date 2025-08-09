"""Extended Kalman Filter (EKF) for simple position and attitude estimation.

State vector ordering: [px, py, pz, vx, vy, vz, phi, theta, psi].

This compact EKF models orientation kinematics and integrates specific force to
propagate linear velocity and position. Two update hooks are provided:
- update_att: fuse absolute attitude (e.g., pitch from accelerometer, yaw from mag)
- update_depth: fuse scalar depth measurement
"""

from typing import Tuple

import numpy as np


class EKF:
    """Minimal EKF with separate attitude and depth updates.

    Parameters
    - Q_scale: scalar process noise multiplier applied to identity
    - R_att: 3x3 measurement covariance for [phi, theta, psi]
    - R_depth: 1x1 (scalar) covariance for depth measurement
    - g: gravity magnitude used to bias-restore accelerometer Z
    """

    def __init__(
        self,
        Q_scale: float = 0.1,
        R_att: np.ndarray = np.diag([0.05, 0.05, 0.05]),
        R_depth: np.ndarray = np.array([[0.5]]),
        g: float = 9.81,
    ) -> None:
        self.x: np.ndarray = np.zeros(9)
        self.P: np.ndarray = np.eye(9) * 0.1
        self.Q: np.ndarray = np.eye(9) * Q_scale
        self.R_att: np.ndarray = R_att
        self.R_depth: np.ndarray = R_depth
        self.g: float = g

    def predict(self, f_b: np.ndarray, omega_b: np.ndarray, dt: float) -> None:
        """Propagate state using body specific force and body rates.

        Parameters
        - f_b: body-frame specific force [ax, ay, az] (m/s^2)
        - omega_b: body rates [p, q, r] (rad/s)
        - dt: timestep (s)
        """
        px, py, pz, vx, vy, vz, phi, th, psi = self.x
        p, q, r = omega_b

        # Orientation kinematics
        phi += (p + np.sin(phi) * np.tan(th) * q + np.cos(phi) * np.tan(th) * r) * dt
        th += (np.cos(phi) * q - np.sin(phi) * r) * dt
        psi += (np.sin(phi) / np.cos(th) * q + np.cos(phi) / np.cos(th) * r) * dt

        # Linear kinematics (specific force + gravity restoration on z)
        ax, ay, az = f_b
        az += self.g
        vx += ax * dt
        vy += ay * dt
        vz += az * dt
        px += vx * dt
        py += vy * dt
        pz += vz * dt

        self.x = np.array([px, py, pz, vx, vy, vz, phi, th, psi])
        self.P = self.P + self.Q * dt

    def update_att(self, att: np.ndarray) -> None:
        """Fuse absolute attitude measurement [phi, theta, psi]."""
        H = np.zeros((3, 9))
        H[:, 6:9] = np.eye(3)
        y = att - self.x[6:9]
        S = H @ self.P @ H.T + self.R_att
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        self.P = (np.eye(9) - K @ H) @ self.P

    def update_depth(self, depth: float) -> None:
        """Fuse scalar depth measurement."""
        H = np.zeros((1, 9))
        H[0, 2] = 1.0
        z = np.array([depth])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_depth
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(9) - K @ H) @ self.P

    @property
    def pos(self) -> np.ndarray:
        """Current position estimate [px, py, pz]."""
        return self.x[0:3]

    @property
    def att(self) -> np.ndarray:
        """Current attitude estimate [phi, theta, psi]."""
        return self.x[6:9]
