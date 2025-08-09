"""3D animation utilities for visualizing the simulated sub trajectory.

Also supports saving separate PNGs for the 3D path and the depth-vs-time plot,
and decimates frames when writing MP4 to keep video length reasonable.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return body-to-world rotation matrix for the given Euler angles."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def animate_3d(times, poses, targets=None, save_path=None, *, save_png_base: str | None = None, show: bool = False, fps: int = 30, event_points: list[tuple] | None = None):
    """Animate 3D trajectory; optionally save MP4 and separate PNGs (3D + depth).

    The animation canvas is 3D-only. Depth is saved to a separate PNG when
    save_png_base is provided.
    """
    # Compute bounds across poses/targets/events and pad, so the full path fits
    xs = [p[0] for p in poses]; ys = [p[1] for p in poses]; zs = [p[2] for p in poses]
    if targets:
        xs += [t[0] for t in targets]; ys += [t[1] for t in targets]; zs += [t[2] for t in targets]
    if event_points:
        xs += [e[0] for e in event_points]; ys += [e[1] for e in event_points]; zs += [e[2] for e in event_points]
    def pad(lo, hi, frac=0.1):
        span = max(1e-6, hi - lo); p = span * frac; return lo - p, hi + p
    xlo, xhi = pad(min(xs) if xs else -10, max(xs) if xs else 60)
    ylo, yhi = pad(min(ys) if ys else -10, max(ys) if ys else 60)
    zlo, zhi = pad(min(zs) if zs else 0, max(zs) if zs else 20)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z (+down)")
    ax.view_init(elev=25, azim=-60)
    ax.set_xlim(xlo, xhi); 
    ax.set_ylim(ylo, yhi); 
    ax.set_zlim(zlo, zhi)
    (traj,) = ax.plot([], [], [], lw=2, label="trajectory")
    (tgt_line,) = ax.plot([], [], [], "o--", alpha=0.4, label="targets") if targets else (None,)
    (body_line,) = ax.plot([], [], [], lw=3)
    # Event points collection (red dots)
    ev_scatter = ax.scatter([], [], [], c='r', s=30, label="events") if event_points else None

    body = np.array([[0.6, 0, 0], [-0.6, 0, 0], [0, 0, 0.15], [0, 0, -0.15]]).T

    def init():
        ax.set_xlim(-10, 60)
        ax.set_ylim(-10, 60)
        ax.set_zlim(0, 20)
        # legends
        ax.legend(loc="upper left")
        return traj, body_line

    def update(i):
        xs = [p[0] for p in poses[: i + 1]]
        ys = [p[1] for p in poses[: i + 1]]
        zs = [p[2] for p in poses[: i + 1]]
        traj.set_data(xs, ys)
        traj.set_3d_properties(zs)
        if targets is not None and len(targets) > 0 and tgt_line is not None:
            tx = [t[0] for t in targets[: i + 1]]
            ty = [t[1] for t in targets[: i + 1]]
            tz = [t[2] for t in targets[: i + 1]]
            tgt_line.set_data(tx, ty)
            tgt_line.set_3d_properties(tz)
        x, y, z, roll, pitch, yaw = poses[i]
        R = rotation_matrix(roll, pitch, yaw)
        wf = (R @ body).T + np.array([x, y, z])
        body_line.set_data(wf[:, 0], wf[:, 1])
        body_line.set_3d_properties(wf[:, 2])
        # Update event points if provided
        if event_points and ev_scatter is not None:
            # show all events up to this frame index i
            ex = [p[0] for p in event_points if True]
            ey = [p[1] for p in event_points if True]
            ez = [p[2] for p in event_points if True]
            ev_scatter._offsets3d = (ex, ey, ez)
        return traj, body_line

    # Decimate frames to achieve target fps in output video (prevents overly long MP4s)
    if len(times) > 1:
        total_T = times[-1] - times[0]
        dt = max(1e-9, total_T / max(1, (len(times) - 1)))
    else:
        dt = 0.05
    stride = max(1, int(round((1.0 / dt) / fps)))
    ani = FuncAnimation(fig, update, frames=range(0, len(times), stride), init_func=init, interval=1000 / max(fps, 1), blit=False)

    # Write MP4
    if save_path:
        try:
            ani.save(save_path, writer="ffmpeg", fps=fps)
        except Exception:
            # Fallback: save last frame PNG
            fig.savefig(save_path.replace(".mp4", ".png"))

    # Save separate PNGs if requested
    if save_png_base:
        # Save the current 3D figure
        fig.savefig(f"{save_png_base}_3d.png", dpi=150)
        # Depth vs time PNG with full x-range and padded y-range
        depth_series = [p[2] for p in poses]
        figd = plt.figure(figsize=(6, 4))
        axd = figd.add_subplot(111)
        axd.set_title("Depth vs Time"); axd.set_xlabel("t (s)"); axd.set_ylabel("depth (m)")
        axd.plot(times, depth_series, label="depth")
        if times:
            axd.set_xlim(times[0], times[-1])
        if depth_series:
            ymin, ymax = min(depth_series), max(depth_series)
            ylo, yhi = pad(ymin, ymax, 0.1)
            axd.set_ylim(ylo, yhi)
        axd.legend(loc="best")
        figd.savefig(f"{save_png_base}_depth.png", dpi=150)
        plt.close(figd)

    if show:
        plt.show()

    return fig, ani
