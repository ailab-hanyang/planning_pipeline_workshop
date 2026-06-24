"""
Cubic spline planner.

Source  : PythonRobotics/PathPlanning/CubicSpline/cubic_spline_planner.py
          (https://github.com/AtsushiSakai/PythonRobotics)
Author  : Atsushi Sakai (@Atsushi_twi)
License : MIT

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import math
import bisect
import numpy as np


class CubicSpline1D:
    def __init__(self, x, y):
        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x must be sorted in ascending order")

        self.x, self.y, self.nx = list(x), list(y), len(x)
        self.a = list(y)

        A = self._calc_A(h)
        B = self._calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        self.b, self.d = [], []
        for i in range(self.nx - 1):
            self.d.append((self.c[i+1] - self.c[i]) / (3.0 * h[i]))
            self.b.append((self.a[i+1] - self.a[i]) / h[i]
                          - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i+1]))

    def calc_position(self, x):
        if x < self.x[0] or x > self.x[-1]:
            return None
        i  = min(bisect.bisect(self.x, x) - 1, self.nx - 2)
        dx = x - self.x[i]
        return self.a[i] + self.b[i]*dx + self.c[i]*dx**2 + self.d[i]*dx**3

    def calc_first_derivative(self, x):
        if x < self.x[0] or x > self.x[-1]:
            return None
        i  = min(bisect.bisect(self.x, x) - 1, self.nx - 2)
        dx = x - self.x[i]
        return self.b[i] + 2.0*self.c[i]*dx + 3.0*self.d[i]*dx**2

    def calc_second_derivative(self, x):
        if x < self.x[0] or x > self.x[-1]:
            return None
        i  = min(bisect.bisect(self.x, x) - 1, self.nx - 2)
        dx = x - self.x[i]
        return 2.0*self.c[i] + 6.0*self.d[i]*dx

    def _calc_A(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != self.nx - 2:
                A[i+1, i+1] = 2.0 * (h[i] + h[i+1])
            A[i+1, i] = h[i]
            A[i, i+1] = h[i]
        A[0, 1] = 0.0
        A[-1, -2] = 0.0
        A[-1, -1] = 1.0
        return A

    def _calc_B(self, h, a):
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i+1] = (3.0*(a[i+2]-a[i+1])/h[i+1]
                      - 3.0*(a[i+1]-a[i])/h[i])
        return B


class CubicSpline2D:
    def __init__(self, x, y):
        self.s  = self._calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def _calc_s(self, x, y):
        dx, dy = np.diff(x), np.diff(y)
        self.ds = np.hypot(dx, dy)
        return [0.0] + list(np.cumsum(self.ds))

    def calc_position(self, s):
        return self.sx.calc_position(s), self.sy.calc_position(s)

    def calc_yaw(self, s):
        return math.atan2(self.sy.calc_first_derivative(s),
                          self.sx.calc_first_derivative(s))

    def calc_curvature(self, s):
        dx  = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy  = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        return (ddy*dx - ddx*dy) / (dx**2 + dy**2)**1.5


def calc_spline_course(x, y, ds=0.1):
    """Interpolate waypoints into a dense spline path."""
    sp = CubicSpline2D(x, y)
    s_vals = list(np.arange(0, sp.s[-1], ds))
    rx, ry, ryaw, rk = [], [], [], []
    for s in s_vals:
        ix, iy = sp.calc_position(s)
        rx.append(ix);  ry.append(iy)
        ryaw.append(sp.calc_yaw(s))
        rk.append(sp.calc_curvature(s))
    return rx, ry, ryaw, rk, s_vals
