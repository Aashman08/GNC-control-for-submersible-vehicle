import numpy as np
from typing import Optional


class PID:
    """
    Proportional-Integral-Derivative (PID) controller.

    Concepts
    - Proportional gain (kp): Multiplies the instantaneous error. Larger kp reacts
      more strongly to error, reducing it faster but risking overshoot.
    - Integral gain (ki): Integrates (sums) the error over time to remove
      steady-state error (bias). Too large ki can "wind up" and cause oscillation.
    - Derivative gain (kd): Reacts to the rate of change of error. It provides
      damping (resistance to rapid changes), helping prevent overshoot and
      oscillations. In control-speak, "gain" simply means multiplier.

    This implementation includes:
    - Integral clamping (i_min/i_max) to prevent windup
    - Output saturation (out_min/out_max)
    - Optional first-order low-pass filter on the derivative term (d_cutoff_hz)
      to reduce noise amplification
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        i_min: float,
        i_max: float,
        out_min: float,
        out_max: float,
        d_cutoff_hz: float = 0.0,
    ) -> None:
        """Initialize PID gains and limits.

        Parameters
        - kp, ki, kd: PID gains (unit-dependent so that the summed output matches
          the actuator command units)
        - i_min, i_max: Bounds for the integral accumulator to prevent windup
        - out_min, out_max: Output saturation (final PID output limits)
        - d_cutoff_hz: First-order low-pass cutoff (Hz) applied to the derivative term to attenuate high-frequency noise. 0 → no filtering (pure derivative).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.i_min = i_min
        self.i_max = i_max
        self.out_min = out_min
        self.out_max = out_max

        self.i: float = 0.0
        self.prev: float = 0.0
        self.dstate: float = 0.0
        self.cut: float = d_cutoff_hz

    def update(self, error: float, dt: float) -> float:
        """Compute PID output given error and timestep.

        Output:
        - kp*error + integral + kd*filtered_derivative

        Notes:
        - Integral term is clamped to [i_min, i_max] (anti-windup).
        - Final output is clamped to [out_min, out_max].
        - Derivative uses an optional first-order low-pass (cutoff = d_cutoff_hz Hz):
            d_raw = (error - prev) / dt
            a = dt / (RC + dt), RC = 1 / (2π·d_cutoff_hz) if d_cutoff_hz > 0 else a = 1
            dstate = (1 - a) * dstate + a * d_raw
        """
        # Integral update with clamping (anti-windup)
        self.i += self.ki * error * dt
        self.i = float(np.clip(self.i, self.i_min, self.i_max))

        # Derivative (raw slope of error) with optional low-pass smoothing
        d_raw = (error - self.prev) / max(dt, 1e-6)  # finite-difference derivative
        if self.cut > 0:
            rc = 1.0 / (2 * np.pi * self.cut)  # RC time constant from cutoff (Hz)
            a = dt / (rc + dt)  # EMA blend factor in [0,1]
        else:
            a = 1.0  # no filtering → use raw derivative
        self.dstate = (1 - a) * self.dstate + a * d_raw  # EMA low-pass of derivative

        # PID sum with saturation
        output = self.kp * error + self.i + self.kd * self.dstate
        output = float(np.clip(output, self.out_min, self.out_max))

        # Store current error for next derivative computation
        self.prev = error
        return output

    def step(self, error: float, dt: float, *, cond_integrate: bool = True) -> float:
        """PID step with optional conditional integration.

        If cond_integrate is False, skip the integral accumulator update for this
        step (useful during transients or when actuator is saturated upstream).
        """
        # Derivative uses previous error regardless of integral gating
        d_raw = (error - self.prev) / max(dt, 1e-6)
        if self.cut > 0:
            rc = 1.0 / (2 * np.pi * self.cut)
            a = dt / (rc + dt)
        else:
            a = 1.0
        self.dstate = (1 - a) * self.dstate + a * d_raw

        if cond_integrate:
            self.i += self.ki * error * dt
            self.i = float(np.clip(self.i, self.i_min, self.i_max))

        output = self.kp * error + self.i + self.kd * self.dstate
        output = float(np.clip(output, self.out_min, self.out_max))
        self.prev = error
        return output
