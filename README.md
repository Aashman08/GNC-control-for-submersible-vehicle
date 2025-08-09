# Submersible MVP Simulator — Beginner Edition

Learn the basics of underwater vehicle control by running a small, laptop-only simulator.
You’ll see how a sub controls **depth** and **heading** using a motor, rudder, and stern fins.

## How it’s organized
- `run_simulation.py` – main entry point (run the demo).
- `submarine_physics.py` – tiny physics model (how the sub moves).
- `sensor_emulator.py` – fake sensors with noise.
- `sensor_filters.py` – smooth out noisy sensor readings.
- `pid_controller.py` – PID control math.
- `actuator_allocator.py` – turn control outputs into rudder/stern/motor commands.
- `safety_checks.py` – simple safety scaffolding.
- `simulation_settings.yaml` – edit setpoints, gains, and dynamics here.

## Install
```bash
python3 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Run the demo
```bash
python -m subsim.run_simulation --duration 120
```

## Tweak something
Open `subsim/simulation_settings.yaml` and try different depth/heading setpoints under `setpoints:`.
