import numpy as np
from typing import Dict, Tuple, Any, Union

from .helper_functions import RateLimiter


class ActuatorAllocator:
    """Map controller outputs to actuator commands with saturation and rate limits.

    - Rudder and stern plane commands are saturated to safe ranges.
    - An optional heave command is passed through unchanged but constrained to [-1, +1].
    - All actuators are passed through rate limiters to emulate actuator slew limits.

    Constructor accepts either a `limits` dictionary directly or a full config
    with a `limits` section.
    """

    def __init__(self, limits: Union[Dict[str, float], Dict[str, Any]]) -> None:
        # Accept either the raw limits or a full cfg containing limits
        lims = limits.get("limits", limits)  # type: ignore[arg-type]
        self.rudder_rl = RateLimiter(lims["rudder_rate"])   # type: ignore[index]
        self.stern_rl = RateLimiter(lims["stern_rate"])     # type: ignore[index]
        self.thrust_rl = RateLimiter(lims["thrust_rate"])   # type: ignore[index]
        self.heave_rl = RateLimiter(lims.get("heave_rate", 1.0))  # optional, default 1.0

        # Saturation bounds
        self.rmin: float = -0.7
        self.rmax: float = 0.7
        self.smin: float = -0.5
        self.smax: float = 0.5

    def allocate(
        self,
        u_yaw: float,
        u_pitch: float,
        thrust_sp: float,
        dt: float,
        *,
        h_cmd: float = 0.0,
    ) -> Tuple[Tuple[float, float, float, Tuple[float]], Dict[str, bool]]:
        """Allocate control efforts to actuators with saturation and rate limits.

        Parameters
        - u_yaw: yaw controller output → rudder command
        - u_pitch: pitch controller output → stern command
        - thrust_sp: desired thrust (0..1)
        - dt: timestep (s)
        - h_cmd: optional heave command in [-1, +1]

        Returns
        - ((rudder, stern, thrust, (heave,)), flags): saturated and rate-limited
          commands and a flags dict indicating whether each channel saturated.
        """
        # Saturate primary surfaces
        rudder_unsat = float(np.clip(u_yaw, self.rmin, self.rmax))
        stern_unsat = float(np.clip(u_pitch, self.smin, self.smax))
        thrust_unsat = float(np.clip(thrust_sp, 0.0, 1.0))
        heave_unsat = float(np.clip(h_cmd, -1.0, 1.0))

        flags = {
            "rudder_sat": rudder_unsat != u_yaw,
            "stern_sat": stern_unsat != u_pitch,
            "thrust_sat": thrust_unsat != thrust_sp,
            "heave_sat": heave_unsat != h_cmd,
        }

        # Rate limit surfaces and thrust
        rudder = self.rudder_rl.step(rudder_unsat, dt)
        stern = self.stern_rl.step(stern_unsat, dt)
        thrust = self.thrust_rl.step(thrust_unsat, dt)
        heave = self.heave_rl.step(heave_unsat, dt)

        return (rudder, stern, thrust, (heave,)), flags
