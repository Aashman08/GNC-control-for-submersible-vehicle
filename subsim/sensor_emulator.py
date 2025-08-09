import time
from typing import Any, Dict

import numpy as np


class SensorEmulator:
    """Generate noisy sensor measurements from ground-truth state."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg["sensors"]
        np.random.seed(cfg["sim"]["seed"])

    def read(self, true_state: Dict[str, float], dt: float) -> Dict[str, Any]:
        """Return a measurement packet or a stale marker (dropout)."""
        if np.random.rand() < self.cfg.get("dropout_prob", 0.0):
            return {"stale": True}

        gyro_z = true_state["yaw_rate"] + np.random.normal(0, self.cfg["imu_gyro_noise"])
        pitch_accel = true_state["pitch"] + np.random.normal(0, self.cfg["imu_accel_noise"])
        yaw_mag = true_state["yaw"] + np.random.normal(0, self.cfg["mag_noise"])
        depth = true_state["depth"] + np.random.normal(0, self.cfg["pressure_noise"])

        return {
            "gyro_z": gyro_z,
            "pitch_accel": pitch_accel,
            "yaw_mag": yaw_mag,
            "depth": depth,
            "stale": False,
            "ts": time.time(),
        }
