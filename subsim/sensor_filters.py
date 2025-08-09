"""Basic complementary filters to smooth yaw and pitch measurements."""

import numpy as np


class ComplementaryYaw:
    """
    Complementary filter for yaw (heading) that fuses:
    - Gyro integration (high-pass behavior): captures fast changes but drifts over time.
    - Magnetometer yaw (low-pass behavior): provides absolute angle but is noisy.

    Update equation (discrete time):
        yaw_estimate_k = (1 - alpha) * (yaw_estimate_{k-1} + gyro_z * dt) + alpha * yaw_mag
    Then wrap the result to [-pi, pi).

    - alpha in [0, 1]: blending factor.
      Smaller alpha favors gyro (more responsive, more drift).
      Larger alpha favors magnetometer (less drift, potentially noisier).
    """

    def __init__(self, alpha: float = 0.02) -> None:
        self.alpha = alpha
        self.yaw_estimate = 0.0
        self.init = False

    def update(self, yaw_mag: float, gyro_z: float, dt: float) -> float:
        """
        Update the yaw estimate.

        Parameters
        - yaw_mag: magnetometer-derived yaw (rad), absolute but noisy
        - gyro_z: yaw rate from gyro (rad/s), responsive but drifty when integrated
        - dt: timestep (s)

        Returns
        - yaw_estimate (rad) wrapped to [-pi, pi)
        """
        if not self.init:
            # Initialize to absolute yaw to avoid large transient from arbitrary start
            self.yaw_estimate = yaw_mag
            self.init = True

        # Integrate gyro rate to predict yaw (dead-reckoning)
        y_gyro = self.yaw_estimate + gyro_z * dt

        # Complementary blend: mostly gyro, corrected by magnetometer
        self.yaw_estimate = (1 - self.alpha) * y_gyro + self.alpha * yaw_mag

        # Keep angle in canonical range
        self.yaw_estimate = (self.yaw_estimate + np.pi) % (2 * np.pi) - np.pi
        return self.yaw_estimate


class ComplementaryPitch:
    """
    Complementary filter for pitch that fuses:
    - Gyro integration about body y-axis (high-pass behavior).
    - Accelerometer-derived pitch (low-pass behavior) assuming gravity reference.

    Update equation (discrete time):
        pitch_estimate_k = (1 - alpha) * (pitch_estimate_{k-1} + gyro_y * dt) + alpha * pitch_accel
    Then wrap the result to [-pi, pi).

    - alpha in [0, 1]: blending factor as above.
    """

    def __init__(self, alpha: float = 0.02) -> None:
        self.alpha = alpha
        self.pitch_estimate = 0.0
        self.init = False

    def update(self, pitch_accel: float, gyro_y: float, dt: float) -> float:
        """
        Update the pitch estimate.

        Parameters
        - pitch_accel: pitch angle from accelerometer (rad), absolute but noisy
        - gyro_y: pitch rate from gyro (rad/s), responsive but drifty when integrated
        - dt: timestep (s)

        Returns
        - pitch_estimate (rad) wrapped to [-pi, pi)
        """
        if not self.init:
            # Initialize to absolute pitch to avoid transient jump
            self.pitch_estimate = pitch_accel
            self.init = True

        # Integrate gyro rate to predict pitch (dead-reckoning)
        p_gyro = self.pitch_estimate + gyro_y * dt

        # Complementary blend: mostly gyro, corrected by accelerometer
        self.pitch_estimate = (1 - self.alpha) * p_gyro + self.alpha * pitch_accel

        # Keep angle in canonical range
        self.pitch_estimate = (self.pitch_estimate + np.pi) % (2 * np.pi) - np.pi
        return self.pitch_estimate
