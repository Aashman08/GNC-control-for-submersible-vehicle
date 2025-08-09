"""Run a full submersible simulation demo with plotting and 3D animation."""
import os
from pathlib import Path

import argparse
from dataclasses import dataclass
from typing import List, Dict

import numpy as np
import yaml
import matplotlib.pyplot as plt

from .submarine_physics import SubmarinePhysics
from .actuator_allocator import ActuatorAllocator
from .pid_controller import PID
from .sensor_emulator import SensorEmulator
from .mode_manager import ModeManager, Mode
from .watchdogs import Watchdogs
from .ekf import EKF
from .scenarios import get_scenario
from .viz3d import animate_3d


def wrap_angle(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def bearing_to(dx: float, dy: float) -> float:
    return np.arctan2(dy, dx)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--scenario",
        default="step_depth",
        help="Scenario name (step_depth, heading_hold_with_current, grid_patrol)",
    )
    ap.add_argument(
        "--save3d", default=None, help="Path to save 3D animation (mp4)."
    )
    ap.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional duration override (s), otherwise scenario default is used.",
    )
    ap.add_argument(
        "--live",
        action="store_true",
        default=True,
        help="Show a realtime 3D view while the simulation runs.",
    )
    args = ap.parse_args()

    cfg_path = Path(__file__).resolve().parent / "simulation_settings.yaml"
    with cfg_path.open("r") as fh:
        cfg = yaml.safe_load(fh)

    scenario = get_scenario(args.scenario)
    print(f"Scenario: {scenario.name}")
    print(f"Sim dt: {cfg['sim']['dt']} s  | Duration: {args.duration or scenario.duration_s} s")
    print(
        "Depth sp rate:", cfg['sim'].get('depth_sp_rate', 0.6),
        "m/s  | Approach radius:", cfg['sim'].get('approach_radius_depth', 8.0), "m",
    )

    plant = SubmarinePhysics(cfg)
    sensors = SensorEmulator(cfg)
    allocator = ActuatorAllocator(cfg["limits"]) if "limits" in cfg else ActuatorAllocator(cfg)
    ekf = EKF()

    # PIDs
    pids = cfg["pid"]
    depth_pid = PID(**pids["depth_outer"])  # outer loop maps depth error to heave
    pitch_pid = PID(**pids["pitch"])        # pitch hold controller
    yaw_pid = PID(**pids["yaw"])            # yaw/heading controller

    # Health/FSM
    @dataclass
    class Health:
        sensors_ready: bool = True
        actuators_ready: bool = True
        emergency: bool = False

    health = Health()
    nav: Dict[str, bool] = {"stable": True, "at_surface": False, "at_base": False}
    mm = ModeManager(health, nav, params={"auto_surface_when_home": False})
    dogs = Watchdogs()

    # Setpoints
    depth_sp = cfg["setpoints"]["depth_m"]
    depth_sp_cmd = depth_sp  # smoothed/rate-limited setpoint actually fed to controller
    yaw_sp = np.deg2rad(cfg["setpoints"]["heading_deg"])
    speed_sp = cfg["setpoints"]["speed_mps"]

    # Waypoint follower (if scenario defines waypoints)
    wps = scenario.waypoints or []
    wp_idx = 0
    arrive_radius = 1.5

    # Logs for 3D viz
    poses: List[tuple] = []
    tgt_traj: List[tuple] = []
    times: List[float] = []
    event_pts: List[tuple] = []  # event markers (x,y,z) at event application time

    # Time-series logs for detailed plots (restore old runner behavior)
    t_hist: List[float] = []
    depth_hist: List[float] = []
    depth_target_hist: List[float] = []
    yaw_hist: List[float] = []
    yaw_target_hist: List[float] = []
    pitch_hist: List[float] = []
    pitch_target_hist: List[float] = []
    rudder_hist: List[float] = []
    stern_hist: List[float] = []
    thrust_hist: List[float] = []
    speed_hist: List[float] = []

    # Live 3D setup (optional)
    live = args.live
    if live:
        plt.ion()
        fig_live = plt.figure(figsize=(6, 6))
        ax_live = fig_live.add_subplot(111, projection="3d")
        ax_live.set_xlabel("X")
        ax_live.set_ylabel("Y")
        ax_live.set_zlabel("Z (+down)")
        ax_live.view_init(elev=25, azim=-60)
        (live_line,) = ax_live.plot([], [], [], lw=2, label="trajectory")
        ax_live.legend(loc="upper left")

    # Loop
    dt = cfg["sim"]["dt"]
    T = args.duration if args.duration is not None else scenario.duration_s
    steps = int(T / dt)
    # Heave PI persistent state
    heave_i = 0.0
    # Pitch setpoint rate limiter state
    last_pitch_sp = 0.0
    status_every_steps = max(1, int(1.0 / dt))  # ~1 Hz log
    print("Starting main loop ...")
    for k in range(steps):
        t = k * dt
        # Apply timed events
        for ev in scenario.events:
            if abs(ev.t - t) < 1e-9:
                # Record an event marker at current position
                event_pts.append((getattr(plant, "x", 0.0), getattr(plant, "y", 0.0), plant.depth))
                depth_sp = ev.kwargs.get("depth_sp", depth_sp)
                heading_deg = ev.kwargs.get("heading_sp_deg", None)
                if heading_deg is not None:
                    yaw_sp = np.deg2rad(heading_deg)
                speed_sp = ev.kwargs.get("speed_mps", speed_sp)
                cfg["sim"]["current_yaw_disturb"] = ev.kwargs.get(
                    "current_yaw_disturb", cfg["sim"].get("current_yaw_disturb", 0.0)
                )
                print(f"t={t:6.2f}s EVENT applied: {ev.kwargs}")

        # If following waypoints, set heading/depth to next wp
        if wps and wp_idx < len(wps):
            target = wps[wp_idx]
            x = getattr(plant, "x", 0.0); y = getattr(plant, "y", 0.0)
            tx, ty = target["x"], target["y"]
            dx, dy = tx - x, ty - y
            dist = (dx * dx + dy * dy) ** 0.5
            # Line-of-sight (LOS) virtual point: aim at a point slightly before the target
            lookahead = float(cfg["sim"].get("los_lookahead_m", 6.0))
            if dist > 1e-6:
                ux, uy = dx / dist, dy / dist
            else:
                ux, uy = 1.0, 0.0
            la = min(lookahead, dist)
            vx, vy = tx - ux * la, ty - uy * la
            yaw_sp = bearing_to(vx - x, vy - y)
            # Change depth only when within approach radius to reduce oscillations between waypoints
            approach_R = float(cfg["sim"].get("approach_radius_depth", 8.0))
            if (dx * dx + dy * dy) ** 0.5 < approach_R:
                depth_sp = target["z"]
            if dist < float(cfg["sim"].get("arrive_radius_xy", 2.5)):
                wp_idx += 1
                print(f"t={t:6.2f}s Waypoint {wp_idx} reached")
        else:
            target = {
                "x": getattr(plant, "x", 0.0) + 10 * np.cos(yaw_sp),
                "y": getattr(plant, "y", 0.0) + 10 * np.sin(yaw_sp),
                "z": depth_sp,
            }

        # Mode
        @dataclass
        class U:
            surface: bool = False
            return_home: bool = False

        mode = mm.step(t, U())

        # Sensors
        true_state = {
            "yaw": plant.yaw,
            "yaw_rate": plant.yaw_rate,
            "pitch": plant.pitch,
            "depth": plant.depth,
        }
        meas = sensors.read(true_state, dt)
        dogs.on_sensor()

        # EKF predict/update (simplified)
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, meas.get("gyro_z", 0.0)]), dt)
        if not meas["stale"]:
            ekf.update_depth(meas["depth"])
            ekf.update_att(np.array([0.0, meas["pitch_accel"], meas["yaw_mag"]]))

        # Rate-limit the commanded depth setpoint so the vehicle holds current depth between waypoints
        rate = float(cfg["sim"].get("depth_sp_rate", 0.6))
        err_sp = depth_sp - depth_sp_cmd
        step_sp = np.clip(err_sp, -rate * dt, rate * dt)
        depth_sp_cmd += step_sp

        # Controllers
        if mode in (Mode.HOLD_DEPTH, Mode.RETURN):
            # Outer loop: map depth error to a pitch setpoint using smoothed setpoint
            # Sign: positive pitch (nose-down) should increase depth in the physics model.
            pitch_sp_raw = -depth_pid.step(depth_sp_cmd - ekf.pos[2], dt, cond_integrate=True)
            # Rate-limit pitch setpoint to avoid aggressive flips (helps reduce bounce)
            p_rate = float(cfg["sim"].get("pitch_sp_rate", 0.15))  # rad/s
            max_dp = p_rate * dt
            dp = np.clip(pitch_sp_raw - last_pitch_sp, -max_dp, max_dp)
            pitch_sp = last_pitch_sp + dp
            last_pitch_sp = pitch_sp
            # Inner loops
            rudder_cmd = yaw_pid.step(wrap_angle(yaw_sp - ekf.att[2]), dt, cond_integrate=True)
            pitch_cmd = pitch_pid.step(pitch_sp - ekf.att[1], dt, cond_integrate=True)
            thrust_cmd = float(np.clip(speed_sp / cfg["submarine"]["thrust_to_speed"], 0.0, 1.0))
            (r, s, tcmd, (h,)), flags = allocator.allocate(
                u_yaw=rudder_cmd, u_pitch=pitch_cmd, thrust_sp=thrust_cmd, dt=dt
            )
        elif mode in (Mode.SURFACE, Mode.EMERGENCY):
            (r, s, tcmd, (h,)), flags = allocator.allocate(0.0, 0.0, 0.0, dt, h_cmd=+1.0)
        else:
            (r, s, tcmd, (h,)), flags = allocator.allocate(0.0, 0.0, 0.0, dt, h_cmd=0.0)

        dogs.on_actuator()

        # Heave control with physics-based feedforward + PID style correction
        heave_error = depth_sp_cmd - plant.depth
        # Deadband around setpoint to prevent chattering
        deadband = 0.15
        heave_err_db = 0.0 if abs(heave_error) < deadband else heave_error
        # Integrator (small) with clamp
        heave_i = float(np.clip(heave_i + 0.03 * heave_err_db * dt, -0.8, 0.8))
        heave_p = 0.16 * heave_err_db
        sub = cfg["submarine"]
        heave_gain = float(sub.get("heave_accel_per_cmd", 1.0))
        heave_ff = 0.0
        if heave_gain > 1e-6:
            # Cancel only restoring and pitch-coupled terms; keep physical damping
            heave_ff = (
                sub.get("buoy_k", 0.0) * plant.depth
                - sub.get("depth_coupling_pitch", 0.0) * plant.pitch
            ) / heave_gain
        # Add small derivative-on-depth (extra damping) via vertical speed
        heave_d_cmd = -0.25 * plant.vertical_speed  # command units
        heave_correction = float(np.clip(heave_ff + heave_p + heave_i + heave_d_cmd, -1.0, 1.0))

        state = plant.step(
            r,
            s,
            tcmd,
            dt,
            yaw_disturb=cfg["sim"].get("current_yaw_disturb", 0.0),
            vertical_disturb=cfg["sim"].get("current_step", 0.0),
            heave_cmd=heave_correction,
        )

        nav["at_surface"] = state["depth"] <= 0.1

        # logs
        poses.append((state["x"], state["y"], state["depth"], 0.0, state["pitch"], state["yaw"]))
        tgt_traj.append((target["x"], target["y"], target["z"]))
        times.append(t)

        # Realtime 3D update with light throttling
        if live and (k % 5 == 0):
            xs = [p[0] for p in poses]
            ys = [p[1] for p in poses]
            zs = [p[2] for p in poses]
            live_line.set_data(xs, ys)
            live_line.set_3d_properties(zs)
            # Update bounds so everything stays in view
            if xs and ys and zs:
                pad = 2.0
                ax_live.set_xlim(min(xs) - pad, max(xs) + pad)
                ax_live.set_ylim(min(ys) - pad, max(ys) + pad)
                ax_live.set_zlim(min(zs) - pad, max(zs) + pad)
            fig_live.canvas.draw_idle()
            plt.pause(0.001)

        if k % status_every_steps == 0:
            yaw_deg = float(np.rad2deg(state['yaw']))
            yaw_sp_deg = float(np.rad2deg(yaw_sp))
            print(
                f"t={t:6.2f}s  depth={state['depth']:+6.2f}m  depth_sp={depth_sp_cmd:+6.2f}m  "
                f"yaw={yaw_deg:+6.1f}° sp={yaw_sp_deg:+6.1f}°  pitch={state['pitch']:+5.2f}rad  "
                f"speed={state['speed']:+5.2f}m/s  mode={mode.name}"
            )

        # time-series logs
        t_hist.append(t)
        depth_hist.append(state["depth"])
        depth_target_hist.append(depth_sp_cmd)
        yaw_hist.append(state["yaw"])  # true yaw
        yaw_target_hist.append(yaw_sp)
        pitch_hist.append(state["pitch"])  # true pitch
        pitch_target_hist.append(0.0)
        rudder_hist.append(r)
        stern_hist.append(s)
        thrust_hist.append(tcmd)
        speed_hist.append(state["speed"])

    print("Simulation complete. Saving outputs ...")
    # Output directory: timestamped subfolder inside runs/
    save_dir = cfg.get("sim", {}).get("save_dir", "runs")
    os.makedirs(save_dir, exist_ok=True)
    outdir = os.path.join(save_dir, __import__("time").strftime("%Y%m%d_%H%M%S"))
    os.makedirs(outdir, exist_ok=True)

    # Depth
    plt.figure(); plt.title("Depth (m)")
    plt.plot(t_hist, depth_hist, label="depth")
    plt.plot(t_hist, depth_target_hist, label="depth_target")
    plt.xlabel("t (s)"); plt.ylabel("depth (m)"); plt.grid(True); plt.legend()
    p1 = os.path.join(outdir, "plot_1.png"); 
    plt.savefig(p1, dpi=150); print("Saved:", p1)

    # Yaw
    plt.figure(); plt.title("Yaw (rad)")
    plt.plot(t_hist, yaw_hist, label="yaw")
    plt.plot(t_hist, yaw_target_hist, label="yaw_target")
    plt.xlabel("t (s)"); plt.ylabel("yaw (rad)"); plt.grid(True); plt.legend()
    p2 = os.path.join(outdir, "plot_2.png");
    plt.savefig(p2, dpi=150); print("Saved:", p2)

    # Pitch
    plt.figure(); plt.title("Pitch (rad)")
    plt.plot(t_hist, pitch_hist, label="pitch")
    plt.plot(t_hist, pitch_target_hist, label="pitch_target")
    plt.xlabel("t (s)"); plt.ylabel("pitch (rad)"); plt.grid(True); plt.legend()
    p3 = os.path.join(outdir, "plot_3.png");
    plt.savefig(p3, dpi=150); print("Saved:", p3)

    # Actuators
    plt.figure(); plt.title("Actuators")
    plt.plot(t_hist, rudder_hist, label="rudder")
    plt.plot(t_hist, stern_hist, label="stern")
    plt.plot(t_hist, thrust_hist, label="thrust")
    plt.xlabel("t (s)"); plt.ylabel("cmd"); plt.grid(True); plt.legend()
    p4 = os.path.join(outdir, "plot_4.png"); 
    plt.savefig(p4, dpi=150); print("Saved:", p4)

    # Speed
    plt.figure(); plt.title("Speed (m/s)")
    plt.plot(t_hist, speed_hist, label="speed")
    plt.xlabel("t (s)"); plt.ylabel("speed (m/s)")
    plt.grid(True); plt.legend()
    p5 = os.path.join(outdir, "plot_5.png"); 
    plt.savefig(p5, dpi=150); print("Saved:", p5)

    # Overview figure (XY + depth)
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    axs[0].plot(times, [p[2] for p in poses], label="depth")
    axs[0].plot(times, depth_target_hist, label="depth_sp")
    axs[0].set_xlabel("t (s)"); axs[0].set_ylabel("depth (m)")
    axs[0].legend(); axs[0].grid(True); axs[0].set_title("Depth")
    axs[1].plot([p[0] for p in poses], [p[1] for p in poses], label="traj")
    axs[1].plot([t[0] for t in tgt_traj], [t[1] for t in tgt_traj], "o--", label="targets", alpha=0.5)
    axs[1].set_xlabel("X"); axs[1].set_ylabel("Y")
    axs[1].legend(); axs[1].grid(True); axs[1].set_title("XY")
    ov = os.path.join(outdir, "overview.png"); 
    plt.tight_layout(); 
    plt.savefig(ov, dpi=150); 
    print("Saved:", ov)

    # 3D animation
    save3d_path = args.save3d if args.save3d else os.path.join(outdir, "traj.mp4")
    png_base = os.path.join(outdir, "traj")
    print("Rendering animation ...")
    animate_3d(times, poses, targets=tgt_traj, save_path=save3d_path, save_png_base=png_base, show=True, fps=30, event_points=event_pts)
    print("Saved plots to:", outdir)

    if live:
        # keep the live window responsive before closing interactive mode
        plt.pause(0.1)
        plt.ioff()


if __name__ == "__main__":
    main()
