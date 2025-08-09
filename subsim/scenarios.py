"""Built-in scenarios and events for the simulator."""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional


@dataclass
class Event:
    """A timed event with keyword arguments applied at time t."""

    t: float
    kwargs: Dict[str, Any]


@dataclass
class Scenario:
    """Scenario definition with optional waypoint route."""

    name: str
    duration_s: float
    events: List[Event]
    waypoints: Optional[List[Dict[str, float]]] = None


def grid_waypoints(nx: int = 3, ny: int = 3, spacing: float = 10.0, depth: float = 8.0) -> List[Dict[str, float]]:
    """Create a grid of waypoints in XY at constant depth."""
    wps: List[Dict[str, float]] = []
    for iy in range(ny):
        for ix in range(nx):
            wps.append({"x": ix * spacing, "y": iy * spacing, "z": depth})
    return wps


def load_scenarios() -> List[Scenario]:
    """Return a list of built-in scenarios."""
    scenarios = [
        Scenario(
            name="step_depth",
            duration_s=180.0,
            events=[
                Event(5, {"depth_sp": 5.0}),
                Event(60, {"depth_sp": 12.0}),
                Event(120, {"depth_sp": 3.0}),
            ],
        ),
        Scenario(
            name="heading_hold_with_current",
            duration_s=180.0,
            events=[
                Event(0, {"heading_sp_deg": 90.0}),
                Event(20, {"current_yaw_disturb": 0.03}),
                Event(100, {"current_yaw_disturb": 0.0}),
            ],
        ),
        Scenario(
            name="grid_patrol",
            duration_s=180.0,
            events=[Event(0, {"speed_mps": 3.0})],
            waypoints=grid_waypoints(nx=3, ny=3, spacing=15.0, depth=8.0),
        ),
    ]
    return scenarios


def get_scenario(name: str) -> Scenario:
    """Look up a scenario by name."""
    for s in load_scenarios():
        if s.name == name:
            return s
    raise KeyError(f"Scenario '{name}' not found")
