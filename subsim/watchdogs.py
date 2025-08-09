"""Simple watchdog timers for sensors and actuators."""

import time


class Watchdogs:
    """Track last sensor/actuator events and raise emergency if timeouts elapse."""

    def __init__(self, sensor_timeout: float = 0.25, actuator_timeout: float = 0.25) -> None:
        now = time.time()
        self.last_sensor: float = now
        self.last_act: float = now
        self.sensor_timeout: float = sensor_timeout
        self.actuator_timeout: float = actuator_timeout
        self.emergency: bool = False
        self.heartbeat: int = 0

    def on_sensor(self) -> None:
        """Record a sensor tick."""
        self.last_sensor = time.time()

    def on_actuator(self) -> None:
        """Record an actuator tick and toggle a heartbeat bit."""
        self.last_act = time.time()
        self.heartbeat ^= 1

    def step(self) -> None:
        """Update timeouts and set emergency flag if any deadline is exceeded."""
        now = time.time()
        if (now - self.last_sensor) > self.sensor_timeout or (now - self.last_act) > self.actuator_timeout:
            self.emergency = True
