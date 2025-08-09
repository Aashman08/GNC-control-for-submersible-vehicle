"""General helper utilities used across the simulator."""

import numpy as np


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


class RateLimiter:
    """Limit the rate-of-change of a signal to emulate actuator slew limits."""

    def __init__(self, rate: float) -> None:
        self.rate: float = rate
        self.prev: float = 0.0
        self.initialized: bool = False

    def step(self, target: float, dt: float) -> float:
        """Move previous value toward target without exceeding rate * dt."""
        if not self.initialized:
            self.prev = target
            self.initialized = True

        delta = target - self.prev
        max_delta = self.rate * dt
        delta = max(-max_delta, min(max_delta, delta))
        self.prev += delta
        return self.prev
