"""
Grid map utilities for planning exercises.

Source  : PythonRobotics (https://github.com/AtsushiSakai/PythonRobotics)
Author  : Atsushi Sakai (@Atsushi_twi)
License : MIT

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import math


def BuildObstacleMap(ox, oy, resolution, robot_radius):
    """
    Build a 2D boolean obstacle map from obstacle point lists.

    Returns
    -------
    obstacle_map : 2D list[bool]
    min_x, min_y, max_x, max_y : float
    x_width, y_width : int
    """
    min_x = round(min(ox))
    min_y = round(min(oy))
    max_x = round(max(ox))
    max_y = round(max(oy))

    x_width = int((max_x - min_x) / resolution) + 1   # +1: 경계(max) 칸까지 포함 (벽도 격자에 잡히도록)
    y_width = int((max_y - min_y) / resolution) + 1

    obstacle_map = [[False] * y_width for _ in range(x_width)]

    for ix in range(x_width):
        x = ix * resolution + min_x
        for iy in range(y_width):
            y = iy * resolution + min_y
            for iox, ioy in zip(ox, oy):
                if math.hypot(iox - x, ioy - y) <= robot_radius:
                    obstacle_map[ix][iy] = True
                    break

    return obstacle_map, min_x, min_y, max_x, max_y, x_width, y_width


def MakeSimpleMap():
    """
    Create the shared simple map used across Day 1 & Day 2 exercises.

    Returns
    -------
    ox, oy : list of float  - obstacle positions
    sx, sy : float          - start position
    gx, gy : float          - goal position
    """
    ox, oy = [], []

    # boundary walls
    for i in range(-10, 60):
        ox.append(float(i)); oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0);    oy.append(float(i))
    for i in range(-10, 61):
        ox.append(float(i)); oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0);   oy.append(float(i))

    # internal walls
    for i in range(-10, 40):
        ox.append(20.0); oy.append(float(i))
    for i in range(0, 40):
        ox.append(40.0); oy.append(60.0 - float(i))

    sx, sy = -5.0, -5.0
    gx, gy = 50.0, 50.0

    return ox, oy, sx, sy, gx, gy
