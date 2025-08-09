## Submersible MVP Simulator

Learn and experiment with underwater vehicle control in a lightweight, laptop‑friendly simulator. It demonstrates closed‑loop control of depth and heading using a motor (thrust), a rudder, and stern planes, with realistic actuator limits, noisy sensors, state estimation, a simple mission mode FSM, and 3D visualization.

### Key features
- Depth hold with outer‑loop depth→pitch and inner‑loop pitch/yaw PID
- Heave control with physics‑based feedforward + PID‑style correction
- Noisy sensors, optional dropouts, and an EKF for attitude/depth fusion
- Actuator saturation and slew‑rate limiting
- Built‑in scenarios: step depth, heading hold with current, grid patrol with waypoints
- Live 3D view and saved plots/videos to `runs/`
- Simple, well‑documented Python modules for easy extension

Note on naming: physics‑style short names were standardized to descriptive names across plant, sensors, and runner (e.g., psi→yaw, r→yaw_rate, theta→pitch, q→pitch_rate, z→depth, w→vertical_speed) for readability [[memory:5554254]].

---

## Quick start

### Install
```bash
python3 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```
Optional: to ensure MP4 export works, install `ffmpeg` (e.g., `brew install ffmpeg` on macOS).

### Run a demo
- 2‑minute depth/heading demo via example script:
```bash
python examples/run_depth_heading.py
```
- Or directly via the package entry:
```bash
python -m subsim.run_simulation --duration 120
```

### Choose a scenario
```bash
python -m subsim.run_simulation --scenario step_depth
python -m subsim.run_simulation --scenario heading_hold_with_current
python -m subsim.run_simulation --scenario grid_patrol
```
Optional video path:
```bash
python -m subsim.run_simulation --save3d runs/demo.mp4
```

The live 3D view is enabled by default during the simulation, and outputs are saved under `runs/YYYYMMDD_HHMMSS/`.

---

## Project layout
- `subsim/run_simulation.py` — main orchestrator and logging/plotting
- `subsim/submarine_physics.py` — kinematic/dynamic model (speed, yaw, pitch, depth)
- `subsim/sensor_emulator.py` — noisy IMU/mag/pressure sensors + dropouts
- `subsim/sensor_filters.py` — complementary filters for yaw and pitch (reference)
- `subsim/ekf.py` — compact EKF; fuses attitude and depth
- `subsim/pid_controller.py` — PID with anti‑windup and derivative low‑pass
- `subsim/actuator_allocator.py` — saturation and slew‑rate limiting for actuators
- `subsim/mode_manager.py` — simple mission FSM (INIT→STABILIZE→HOLD_DEPTH, etc.)
- `subsim/scenarios.py` — predefined scenarios, timed events, and waypoint grids
- `subsim/viz3d.py` — 3D animation and optional MP4/PNG export
- `subsim/simulation_settings.yaml` — all tuning/config in one place
- `examples/run_depth_heading.py` — convenience launcher (120 s demo)
- `runs/` — auto‑created output folders with plots, animations, and images

---

## How it works

### Orchestration (`run_simulation.py`)
- Loads `simulation_settings.yaml` and a `Scenario` (`--scenario`).
- Instantiates:
  - `SubmarinePhysics` (the “plant”)
  - `SensorEmulator`
  - `PID` controllers: yaw, pitch, and depth_outer (depth→pitch setpoint)
  - `ActuatorAllocator` (saturation + rate limits + heave)
  - `EKF` (predict with gyro; updates with depth and [pitch, yaw])
  - `ModeManager` (FSM) and `Watchdogs` (basic liveness tracking)
- Waypoint follower (if scenario provides waypoints):
  - Computes a line‑of‑sight heading with configurable lookahead.
  - Changes depth only within an approach radius to reduce ping‑ponging.
- Depth command smoothing:
  - Rate‑limits commanded depth (`depth_sp_cmd`) so the vehicle “holds” between waypoints and transitions smoothly.
- Control loops per step:
  - Outer loop: depth error → pitch setpoint via `depth_outer` PID, then pitch setpoint is rate‑limited.
  - Inner loops: yaw PID → rudder; pitch PID → stern; thrust set by speed setpoint via `thrust_to_speed`.
  - Allocator saturates and slew‑limits rudder/stern/thrust; heave gets its own rate limit and clamp.
- Heave control (vertical acceleration command):
  - Physics‑aware feedforward cancels restoring buoyancy and pitch‑coupling terms (but keeps damping).
  - Small PI(+D‑on‑depth) correction using `depth - depth_sp_cmd` with deadband to avoid chatter.
- Plant integration:
  - Applies yaw and vertical disturbances from the scenario, steps dynamics, updates logs.
- Outputs:
  - Live 3D trajectory view (on by default).
  - Saved plots: depth, yaw, pitch, actuators, speed, overview (XY + depth).
  - 3D animation MP4 (or PNG fallback) and standalone PNGs.

### Plant model (`submarine_physics.py`)
State: `speed`, `yaw`, `yaw_rate`, `pitch`, `pitch_rate`, `depth` (+down), `vertical_speed`, and XY positions for viz.
- Speed: first‑order lag toward a thrust‑determined steady‑state speed capped by `speed_max` (time constant `speed_tau`).
- Yaw: acceleration from rudder scales with speed; linear damping; optional external yaw disturbance; angles wrapped to \[-π, π).
- Pitch: acceleration from stern scales with speed; linear damping; angles wrapped to \[-π, π).
- Vertical (heave): second‑order with restoring term (`buoy_k`), damping (`heave_damp`), pitch‑coupled vertical accel (`depth_coupling_pitch`), commanded heave accel (`heave_accel_per_cmd`), and external vertical disturbance. Depth increases downward.

### Sensors and estimation
- `sensor_emulator.py`: returns `gyro_z`, `pitch_accel` (accel‑derived pitch), `yaw_mag` (mag yaw), and `depth`, with Gaussian noise and optional dropouts.
- `ekf.py`: 9‑state EKF integrates body rates for attitude and fuses absolute attitude and depth:
  - Predict: uses body rates; integrates attitude and linear kinematics.
  - Update: `update_att([roll, pitch, yaw])`, `update_depth(depth)`.
  - Provides `pos` and `att` properties for controller use.

### Control and allocation
- `pid_controller.py`: PID with integral clamping, output saturation, and optional derivative low‑pass. Also provides `step(..., cond_integrate=True)` to gate integrator.
- `actuator_allocator.py`: clamps rudder/stern/thrust into safe ranges and rate‑limits them. Heave command is clamped/rate‑limited separately.

### Modes and safety
- `mode_manager.py`: INIT→STABILIZE→HOLD_DEPTH; transitions to SURFACE/RETURN/EMERGENCY based on health/nav/commands.
- `watchdogs.py`: tracks sensor/actuator activity; can raise emergency on timeouts.
- `safety_checks.py`: helper to check staleness of sensor data.

### Scenarios and events
- `scenarios.py`:
  - `step_depth`: timed depth setpoint steps.
  - `heading_hold_with_current`: hold heading; applies/removes a yaw disturbance.
  - `grid_patrol`: XY grid waypoints at constant depth with LOS guidance; raises speed.
- Each `Event(t, kwargs)` can change `depth_sp`, `heading_sp_deg`, `speed_mps`, `current_yaw_disturb`, etc. Waypoints include `{x, y, z}`.

### Visualization
- `viz3d.py`: draws the 3D trajectory and a simple sub body frame; can save MP4 and separate PNGs (`..._3d.png`, `..._depth.png`). Event points show up as red markers.

---

## Configuration (`subsim/simulation_settings.yaml`)

### sim
- **dt**: timestep (s)
- **seed**: RNG seed
- **save_dir**: output base directory (default `runs`)
- **current_step**: vertical accel disturbance (m/s^2)
- **current_yaw_disturb**: yaw accel disturbance (rad/s^2)
- **depth_sp_rate**: max change rate for depth setpoint smoothing (m/s)
- Waypoint/XY:
  - **pitch_sp_rate**: max change rate for pitch setpoint (rad/s)
  - **los_lookahead_m**: lookahead distance for LOS guidance (m)
  - **arrive_radius_xy**: arrival radius (m)
  - **approach_radius_depth**: start changing depth when within this distance (m)

### submarine
- **speed_tau** (s): first‑order time constant for speed response
- **speed_max** (m/s): hard speed cap
- **thrust_to_speed** (m/s per cmd): maps thrust command to steady‑state speed
- **yaw_accel_per_rudder** (rad/s^2 per cmd @ 1 m/s), **yaw_damp** (1/s)
- **pitch_accel_per_stern** (rad/s^2 per cmd @ 1 m/s), **pitch_damp** (1/s)
- **buoy_k** (1/s^2): vertical restoring toward 0 depth
- **depth_coupling_pitch** (m/s^2 per rad): vertical accel from pitch angle
- **heave_accel_per_cmd** (m/s^2) and **heave_damp** (1/s)

### sensors
- **imu_gyro_noise** (rad/s), **imu_accel_noise** (rad), **mag_noise** (rad), **pressure_noise** (m)
- **dropout_prob**: probability a read returns stale

### pid
- `yaw`, `pitch`, `depth_outer`: each has
  - **kp, ki, kd**
  - **i_min, i_max**: integral clamp
  - **out_min, out_max**: output clamp
  - **d_cutoff_hz**: derivative low‑pass cutoff (Hz)

### limits
- **rudder_rate**, **stern_rate**, **thrust_rate**: max command change per second

### setpoints
- **depth_m** (m, +down), **heading_deg** (deg), **speed_mps** (m/s)

---

## CLI reference
```text
python -m subsim.run_simulation [--scenario NAME] [--duration SEC] [--save3d PATH] [--live]
```
- **--scenario**: step_depth | heading_hold_with_current | grid_patrol
- **--duration**: overrides scenario duration
- **--save3d**: MP4 output path (also saves PNGs)
- **--live**: show live 3D during run (enabled by default)

Outputs saved to `runs/YYYYMMDD_HHMMSS/`:
- `plot_1.png` (depth), `plot_2.png` (yaw), `plot_3.png` (pitch), `plot_4.png` (actuators), `plot_5.png` (speed), `overview.png` (summary), `traj.mp4` (3D animation).

---

## Extending

- New scenario: add to `load_scenarios()` in `subsim/scenarios.py` or create waypoints with `grid_waypoints(...)`.
- New controller: reuse `PID` or add your own module; wire it in `run_simulation.py`.
- New sensor model: extend `sensor_emulator.py` and update the EKF hooks as needed.
- New visualizations: add plots in `run_simulation.py` or expand `viz3d.py`.

---

## Conventions and glossary

- Axes: X/Y horizontal plane; Z is depth, positive down.
- Angles: yaw (heading, \[-π, π)), pitch (\[-π, π)); angle wrapping keeps values bounded.
- Heave: vertical acceleration command scaled by `heave_accel_per_cmd`.
- Actuator units: rudder/stern/heave are dimensionless commands typically in \[-1, +1]; thrust is \[0, 1].
- Rate limiting: first‑order limiter ensures commands change no faster than specified per‑second rates.

---

## Troubleshooting

- No MP4 saved: ensure `ffmpeg` is installed; PNG fallback should still save the last frame.
- Empty outputs: check write permissions to `runs/`; confirm virtualenv and dependencies are active.
- Oscillations or overshoot: lower `kp` or increase `kd`; increase damping in `submarine` block.
- Slow response: reduce `speed_tau`, increase controller `kp`, or raise limits’ slew rates.
