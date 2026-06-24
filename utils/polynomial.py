"""
Polynomial trajectory primitives for Frenet optimal trajectory planning.

- QuinticPolynomial : 5th-order, fixes start/end position, velocity, acceleration.
                      Used for lateral d(t) (jerk-optimal lane change).
- QuarticPolynomial : 4th-order, fixes start pos/vel/acc and end vel/acc
                      (end position free). Used for longitudinal s(t) velocity keeping.

Source    : PythonRobotics/PathPlanning/FrenetOptimalTrajectory
            (https://github.com/AtsushiSakai/PythonRobotics)
Author    : Atsushi Sakai (@Atsushi_twi) / License: MIT

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import numpy as np


class QuinticPolynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # 시작 조건으로 a0, a1, a2 직접 결정
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        # 종료 조건(위치/속도/가속도)으로 a3, a4, a5 를 선형시스템으로 풀이
        A = np.array([[time**3,      time**4,       time**5],
                      [3*time**2,  4*time**3,    5*time**4],
                      [6*time,     12*time**2,   20*time**3]])
        b = np.array([xe  - self.a0 - self.a1*time - self.a2*time**2,
                      vxe - self.a1 - 2*self.a2*time,
                      axe - 2*self.a2])
        x = np.linalg.solve(A, b)
        self.a3, self.a4, self.a5 = x[0], x[1], x[2]

    def calc_point(self, t):
        return (self.a0 + self.a1*t + self.a2*t**2
                + self.a3*t**3 + self.a4*t**4 + self.a5*t**5)

    def calc_first_derivative(self, t):
        return (self.a1 + 2*self.a2*t + 3*self.a3*t**2
                + 4*self.a4*t**3 + 5*self.a5*t**4)

    def calc_second_derivative(self, t):
        return 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2 + 20*self.a5*t**3

    def calc_third_derivative(self, t):
        return 6*self.a3 + 24*self.a4*t + 60*self.a5*t**2


class QuarticPolynomial:
    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # 시작 조건으로 a0, a1, a2 직접 결정
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        # 종료 속도/가속도만 지정 (종료 위치는 자유) -> a3, a4
        A = np.array([[3*time**2, 4*time**3],
                      [6*time,   12*time**2]])
        b = np.array([vxe - self.a1 - 2*self.a2*time,
                      axe - 2*self.a2])
        x = np.linalg.solve(A, b)
        self.a3, self.a4 = x[0], x[1]

    def calc_point(self, t):
        return self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4

    def calc_first_derivative(self, t):
        return self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3

    def calc_second_derivative(self, t):
        return 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2

    def calc_third_derivative(self, t):
        return 6*self.a3 + 24*self.a4*t
