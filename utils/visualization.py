"""
Common visualization helpers for planning exercises.

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from utils.vehicle_model import plot_car, move, pure_pursuit_steer


def plot_grid_map(ox, oy, sx, sy, gx, gy, ax=None):
    """Plot obstacles, start, and goal on a grid map."""
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 7))
    ax.plot(ox, oy, '.k', ms=2, label='Obstacle')
    ax.plot(sx, sy, 'og', ms=10, label='Start')
    ax.plot(gx, gy, 'xb', ms=10, mew=2, label='Goal')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left')
    return ax


def plot_search_result(closed_x, closed_y, path_x, path_y,
                       ox, oy, sx, sy, gx, gy, title='Search Result'):
    """Show explored nodes (left) and final path (right) side by side."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    for ax in axes:
        ax.plot(ox, oy, '.k', ms=2)
        ax.plot(sx, sy, 'og', ms=10)
        ax.plot(gx, gy, 'xb', ms=10, mew=2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

    axes[0].plot(closed_x, closed_y, 'xc', ms=3, alpha=0.5, label='Explored')
    axes[0].set_title('Explored nodes')
    axes[0].legend()

    axes[1].plot(closed_x, closed_y, 'xc', ms=3, alpha=0.3)
    axes[1].plot(path_x, path_y, '-r', lw=2, label='Path')
    axes[1].set_title(title)
    axes[1].legend()

    plt.tight_layout()
    return fig


def generate_candidate(rx, ry, ryaw, d_offset):
    """Shift a reference path by d_offset along its normal direction (Frenet -> Cartesian)."""
    cand_x = [x - math.sin(yaw) * d_offset for x, yaw in zip(rx, ryaw)]
    cand_y = [y + math.cos(yaw) * d_offset for y, yaw in zip(ry, ryaw)]
    return cand_x, cand_y


def is_collision_free(cand_x, cand_y, obs_x, obs_y, radius):
    """Return True if every path point keeps at least `radius` from all obstacles."""
    for px, py in zip(cand_x, cand_y):
        for ox_, oy_ in zip(obs_x, obs_y):
            if math.hypot(px - ox_, py - oy_) < radius:
                return False
    return True


def plot_obstacles(ax, obs_x, obs_y, radius, label='Obstacle'):
    """Draw obstacles as translucent collision circles plus center markers."""
    for k, (ox_, oy_) in enumerate(zip(obs_x, obs_y)):
        ax.add_patch(plt.Circle((ox_, oy_), radius, color='red', alpha=0.22, zorder=1))
    ax.plot(obs_x, obs_y, 'rx', ms=8, mew=2, zorder=3,
            label=f'{label} (r={radius}m)')


def animate_vehicle_tracking(path_x, path_y, ox, oy,
                              speed=10.0, look_ahead=8.0,
                              title='Vehicle Tracking'):
    """
    Create a matplotlib FuncAnimation of a vehicle following path_x/path_y.

    Parameters
    ----------
    path_x, path_y : list  - path to follow
    ox, oy         : list  - obstacle positions
    speed          : float - constant speed [m/s]
    look_ahead     : float - pure pursuit look-ahead distance [m]
    """
    x   = path_x[0]
    y   = path_y[0]
    yaw = math.atan2(path_y[1] - path_y[0], path_x[1] - path_x[0])
    dt  = 0.1
    step = speed * dt

    rem_x, rem_y = list(path_x), list(path_y)
    frames = []

    for _ in range(3000):
        if len(rem_x) < 2:
            break
        steer, idx = pure_pursuit_steer(x, y, yaw, rem_x, rem_y, look_ahead)
        x, y, yaw  = move(x, y, yaw, step, steer)
        rem_x = rem_x[max(0, idx - 1):]
        rem_y = rem_y[max(0, idx - 1):]
        frames.append((x, y, yaw, list(frames[-1][3] + [x]) if frames else [x],
                       list(frames[-1][4] + [y]) if frames else [y]))
        if math.hypot(path_x[-1] - x, path_y[-1] - y) < 2.0:
            break

    # rebuild history properly
    hx, hy = [path_x[0]], [path_y[0]]
    clean_frames = []
    for fx, fy, fyaw, _, _ in frames:
        hx.append(fx); hy.append(fy)
        clean_frames.append((fx, fy, fyaw, list(hx), list(hy)))

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(ox, oy, '.k', ms=2)
    ax.plot(path_x, path_y, '--', color='gray', lw=1, label='Path')
    ax.plot(path_x[0],  path_y[0],  'og', ms=10, label='Start')
    ax.plot(path_x[-1], path_y[-1], 'xb', ms=10, mew=2, label='Goal')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title(title)
    ax.legend(loc='upper left')

    trail_line, = ax.plot([], [], '-b', lw=1.5, alpha=0.7)
    car_lines   = []

    def update(frame):
        nonlocal car_lines
        fx, fy, fyaw, hx_f, hy_f = frame
        for ln in car_lines:
            ln.remove()
        car_lines = []
        trail_line.set_data(hx_f, hy_f)
        # draw car manually to capture returned lines
        c, s = math.cos(fyaw), math.sin(fyaw)
        rot  = np.array([[c, -s], [s, c]])
        from utils.vehicle_model import VRX, VRY, LF
        pts  = rot @ np.array([VRX, VRY])
        ln,  = ax.plot(pts[0] + fx, pts[1] + fy, color='royalblue', lw=1.5)
        car_lines.append(ln)
        return trail_line,

    ani = animation.FuncAnimation(fig, update, frames=clean_frames,
                                   interval=50, blit=False, repeat=False)
    return ani
