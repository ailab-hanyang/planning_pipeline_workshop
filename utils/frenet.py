"""
Cartesian <-> Frenet coordinate converter.

Source    : PythonRobotics/PathPlanning/FrenetOptimalTrajectory/cartesian_frenet_converter.py
            (https://github.com/AtsushiSakai/PythonRobotics)
Author    : Wang Zheng (@Aglargil)
Reference : Werling et al., "Optimal Trajectory Generation for Dynamic Street
            Scenarios in a Frenet Frame", ICRA 2010.
License   : MIT

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import math


class FrenetConverter:

    @staticmethod
    def cartesian_to_frenet(rs, rx, ry, rtheta, rkappa, rdkappa,
                             x, y, v, a, theta, kappa):
        """Convert Cartesian state to Frenet (s, d) state."""
        dx = x - rx
        dy = y - ry
        cos_r = math.cos(rtheta)
        sin_r = math.sin(rtheta)

        d = math.copysign(math.hypot(dx, dy), cos_r * dy - sin_r * dx)

        delta_theta    = theta - rtheta
        tan_dt         = math.tan(delta_theta)
        cos_dt         = math.cos(delta_theta)
        one_minus_kr_d = 1.0 - rkappa * d

        d_dot  = one_minus_kr_d * tan_dt
        kr_d_p = rdkappa * d + rkappa * d_dot
        d_ddot = (-kr_d_p * tan_dt
                  + one_minus_kr_d / cos_dt**2
                  * (kappa * one_minus_kr_d / cos_dt - rkappa))

        s      = rs
        s_dot  = v * cos_dt / one_minus_kr_d
        dt_p   = one_minus_kr_d / cos_dt * kappa - rkappa
        s_ddot = (a * cos_dt - s_dot**2 * (d_dot * dt_p - kr_d_p)) / one_minus_kr_d

        return [s, s_dot, s_ddot], [d, d_dot, d_ddot]

    @staticmethod
    def frenet_to_cartesian(rs, rx, ry, rtheta, rkappa, rdkappa,
                             s_condition, d_condition):
        """Convert Frenet (s, d) state to Cartesian state."""
        cos_r = math.cos(rtheta)
        sin_r = math.sin(rtheta)

        x = rx - sin_r * d_condition[0]
        y = ry + cos_r * d_condition[0]

        one_minus_kr_d = 1.0 - rkappa * d_condition[0]
        delta_theta    = math.atan2(d_condition[1], one_minus_kr_d)
        cos_dt         = math.cos(delta_theta)
        tan_dt         = d_condition[1] / one_minus_kr_d

        theta  = _normalize(delta_theta + rtheta)
        kr_d_p = rdkappa * d_condition[0] + rkappa * d_condition[1]
        kappa  = (((d_condition[2] + kr_d_p * tan_dt) * cos_dt**2)
                  / one_minus_kr_d + rkappa) * cos_dt / one_minus_kr_d

        d_dot  = d_condition[1] * s_condition[1]
        v      = math.sqrt(one_minus_kr_d**2 * s_condition[1]**2 + d_dot**2)
        dt_p   = one_minus_kr_d / cos_dt * kappa - rkappa
        a      = (s_condition[2] * one_minus_kr_d / cos_dt
                  + s_condition[1]**2 / cos_dt
                  * (d_condition[1] * dt_p - kr_d_p))

        return x, y, theta, kappa, v, a


def _normalize(angle):
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi
