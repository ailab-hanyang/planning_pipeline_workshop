"""
Simple vehicle model and drawing utilities for planning exercises.

Source  : PythonRobotics/PathPlanning/HybridAStar/car.py
          (https://github.com/AtsushiSakai/PythonRobotics)
Author  : Zheng Zh (@Zhengzh)
License : MIT

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import math
import numpy as np
import matplotlib.pyplot as plt

# Vehicle geometry [m]
WB  = 3.0    # wheelbase (rear axle to front axle)
W   = 2.0    # width
LF  = 3.3    # rear axle to front bumper
LB  = 1.0    # rear axle to rear bumper

# Rectangle vertices relative to rear axle center
VRX = [LF,   LF,  -LB, -LB,  LF]
VRY = [W/2, -W/2, -W/2, W/2, W/2]


def plot_car(x, y, yaw, ax=None, color='black', alpha=1.0):
    """Draw a rectangle representing the vehicle at (x, y, yaw)."""
    if ax is None:
        ax = plt.gca()
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    pts = rot @ np.array([VRX, VRY])
    ax.plot(pts[0] + x, pts[1] + y, color=color, alpha=alpha, lw=1.5)
    ax.annotate('', xy=(x + LF * c, y + LF * s), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.5, alpha=alpha))


def move(x, y, yaw, distance, steer, L=WB):
    """Kinematic bicycle model step."""
    x   += distance * math.cos(yaw)
    y   += distance * math.sin(yaw)
    yaw  = (yaw + distance * math.tan(steer) / L + math.pi) % (2 * math.pi) - math.pi
    return x, y, yaw


def pure_pursuit_steer(x, y, yaw, path_x, path_y, look_ahead=6.0):
    """
    Minimal pure-pursuit controller.

    Returns
    -------
    steer : float  - steering angle [rad]
    target_idx : int
    """
    target_idx = len(path_x) - 1
    for i in range(len(path_x)):
        if math.hypot(path_x[i] - x, path_y[i] - y) >= look_ahead:
            target_idx = i
            break
    alpha = math.atan2(path_y[target_idx] - y, path_x[target_idx] - x) - yaw
    steer = math.atan2(2.0 * WB * math.sin(alpha), look_ahead)
    steer = max(-0.6, min(0.6, steer))
    return steer, target_idx
