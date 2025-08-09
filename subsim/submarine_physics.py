import numpy as np
from typing import Any, Dict


class SubmarinePhysics:
    """
    Lightweight kinematic/dynamic model of a submarine for simulation.

    State variables:
        - speed (m/s): forward speed
        - yaw (rad): yaw angle
        - yaw_rate (rad/s): yaw rate
        - pitch (rad): pitch angle
        - pitch_rate (rad/s): pitch rate
        - depth (m): depth (+down)
        - vertical_speed (m/s): vertical velocity (+down)

    Control inputs:
        - rudder_cmd: rudder deflection command (typ. -1..1)
        - stern_cmd: stern plane deflection command (typ. -1..1)
        - thrust_cmd: thrust command (0..1)

    Configuration fields (from cfg['submarine']):
        - speed_tau (s): Speed time constant. First-order response time; smaller
          means faster acceleration towards the commanded steady-state speed.
          It's how quickly speed “catches up” to its target in a first-order (single-pole) model.
        - speed_max (m/s): Cap on forward speed.
        - thrust_to_speed (m/s per cmd): Maps thrust command (0..1) to the
          steady-state speed target before speed_max saturation. It is essentially a conversion factor.
        - yaw_accel_per_rudder (rad/s^2 per cmd): Yaw acceleration per unit
          rudder command at 1 m/s forward speed (scales with speed).
        - yaw_damp (1/s): Linear damping on yaw_rate ("damping" resists motion,
          reducing oscillations and overshoot; larger values slow rotation more).
        - pitch_accel_per_stern (rad/s^2 per cmd): Pitch acceleration per unit
          stern plane command at 1 m/s forward speed (scales with speed).
        - pitch_damp (1/s): Linear damping on pitch_rate.
        - buoy_k (1/s^2): Vertical restoring term pulling depth toward zero
          (e.g., buoyancy/spring-like effect).
        - depth_coupling_pitch (m/s^2 per rad): Vertical acceleration produced
          by pitch angle (e.g., lift component).
    """

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.c: Dict[str, Any] = cfg["submarine"]

        # State
        self.speed: float = 0.0
        self.yaw: float = 0.0
        self.yaw_rate: float = 0.0
        self.pitch: float = 0.0
        self.pitch_rate: float = 0.0
        self.depth: float = 0.0
        self.vertical_speed: float = 0.0

        # Last applied controls (for introspection)
        self.rudder_cmd: float = 0.0
        self.stern_cmd: float = 0.0
        self.thrust_cmd: float = 0.0

    def step(
        self,
        rudder_cmd: float,
        stern_cmd: float,
        thrust_cmd: float,
        dt: float,
        yaw_disturb: float = 0.0,
        vertical_disturb: float = 0.0,
        heave_cmd: float = 0.0,
    ) -> Dict[str, float]:
        """Advance the submarine by ``dt`` seconds and return the updated state.

        Parameters
        - dt: Timestep in seconds
        - rudder_cmd: Rudder command
        - stern_cmd: Stern plane command
        - thrust_cmd: Thrust command (0..1)
        - yaw_disturb: External yaw acceleration disturbance (rad/s^2)
        - vertical_disturb: External vertical acceleration disturbance (m/s^2)
        """
        c = self.c
        self.rudder_cmd = rudder_cmd
        self.stern_cmd = stern_cmd
        self.thrust_cmd = thrust_cmd

        # Map thrust command [0..1] to target steady-state speed (m/s), capped by speed_max
        steady_state_speed = min(c["thrust_to_speed"] * max(thrust_cmd, 0.0), c["speed_max"])
        # First-order lag toward target speed with time constant speed_tau
        self.speed += (steady_state_speed - self.speed) * (dt / max(c["speed_tau"], 1e-6))

        # Yaw dynamics:
        #   yaw_acc = K_rudder * rudder_cmd * speed  -  yaw_damp * yaw_rate  +  disturbance
        yaw_acc = (
            c["yaw_accel_per_rudder"] * self.rudder_cmd * max(self.speed, 0.1)  # control-induced (scales with speed)
            - c["yaw_damp"] * self.yaw_rate                                     # linear damping on yaw rate
            + yaw_disturb                                                       # external yaw accel (rad/s^2)
        )
        self.yaw_rate += yaw_acc * dt        # integrate acceleration → yaw rate
        self.yaw += self.yaw_rate * dt       # integrate rate → yaw angle
        self.yaw = (self.yaw + np.pi) % (2 * np.pi) - np.pi  # wrap to [-pi, pi)

        # Pitch dynamics:
        #   pitch_acc = K_stern * stern_cmd * speed  -  pitch_damp * pitch_rate
        pitch_acc = (
            c["pitch_accel_per_stern"] * self.stern_cmd * max(self.speed, 0.1)   # control-induced (scales with speed)
            - c["pitch_damp"] * self.pitch_rate                                  # linear damping on pitch rate
        )
        self.pitch_rate += pitch_acc * dt     # integrate acceleration → pitch rate
        self.pitch += self.pitch_rate * dt    # integrate rate → pitch angle
        self.pitch = (self.pitch + np.pi) % (2 * np.pi) - np.pi  # wrap to [-pi, pi)

        # Vertical (heave) dynamics (second order with damping):
        #   v_acc = -buoy_k * depth  - heave_damp * vertical_speed
        #            +  depth_coupling_pitch * pitch  +  heave  + disturbance
        v_acc = (
            -c["buoy_k"] * self.depth                        # restoring toward depth = 0
            - float(c.get("heave_damp", 0.8)) * self.vertical_speed  # linear damping on vertical speed
            + c["depth_coupling_pitch"] * self.pitch         # pitch-induced vertical accel
            + float(c.get("heave_accel_per_cmd", 0.0)) * float(np.clip(heave_cmd, -1.0, 1.0))
            + vertical_disturb                               # external vertical accel (m/s^2)
        )
        self.vertical_speed += v_acc * dt       # integrate vertical acceleration → vertical speed
        self.depth += self.vertical_speed * dt  # integrate vertical speed → depth (+down)

        # Placeholder XY kinematics for visualization continuity
        # Advance x, y using a simple holonomic approximation tied to speed and yaw
        dx = self.speed * np.cos(self.yaw) * dt
        dy = self.speed * np.sin(self.yaw) * dt
        self.x = getattr(self, "x", 0.0) + dx
        self.y = getattr(self, "y", 0.0) + dy

        return {
            "speed": self.speed,
            "yaw": self.yaw,
            "yaw_rate": self.yaw_rate,
            "pitch": self.pitch,
            "pitch_rate": self.pitch_rate,
            "depth": self.depth,
            "vertical_speed": self.vertical_speed,
            "x": self.x,
            "y": self.y,
        }
