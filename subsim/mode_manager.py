"""Simple mission mode manager finite-state machine (FSM)."""

from enum import Enum, auto
from typing import Any


class Mode(Enum):
    """Mission modes for the submersible."""

    INIT = auto()
    STABILIZE = auto()
    HOLD_DEPTH = auto()
    RETURN = auto()
    SURFACE = auto()
    EMERGENCY = auto()


class ModeManager:
    """Transition between modes based on health, navigation, and user commands."""

    def __init__(self, health: Any, nav: dict, params: dict) -> None:
        self.mode: Mode = Mode.INIT
        self.health = health
        self.nav = nav
        self.p = params

    def step(self, t: float, user_cmd: Any) -> Mode:
        """Advance FSM one step and return the active mode."""
        if self.health.emergency:
            self.mode = Mode.EMERGENCY
            return self.mode

        if self.mode == Mode.INIT:
            if self.health.sensors_ready and self.health.actuators_ready:
                self.mode = Mode.STABILIZE
        elif self.mode == Mode.STABILIZE:
            if self.nav.get("stable", True):
                self.mode = Mode.HOLD_DEPTH
        elif self.mode == Mode.HOLD_DEPTH:
            if getattr(user_cmd, "surface", False):
                self.mode = Mode.SURFACE
            elif getattr(user_cmd, "return_home", False):
                self.mode = Mode.RETURN
        elif self.mode == Mode.RETURN:
            if self.nav.get("at_base", False):
                self.mode = (
                    Mode.SURFACE
                    if self.p.get("auto_surface_when_home", False)
                    else Mode.HOLD_DEPTH
                )
        elif self.mode == Mode.SURFACE:
            if self.nav.get("at_surface", False):
                self.mode = Mode.HOLD_DEPTH
        return self.mode
