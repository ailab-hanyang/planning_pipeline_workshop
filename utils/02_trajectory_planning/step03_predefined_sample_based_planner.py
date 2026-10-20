"""
Visualization helpers for step03_predefined_sample_based_planner notebook.

이 실습은 '예측 가능한 패턴(predefined pattern)' 기반 sample-based planner 를 다룬다.
  - 종방향 s(t) 는 하나의 프로파일(CV)로 고정
  - 횡방향 목표 offset d_i 만 여러 개 sampling -> 각 d_i 마다 5차 다항식 d(s) 생성
  - Frenet (s, d) -> Cartesian (x, y) 변환은 utils 의 FrenetConverter 를 사용

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import os
import sys
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from step01_cubic_spline_path import CubicSpline2D


# ==================== 핵심 구현 ====================
# Cartesian <-> Frenet coordinate converter.
# Source    : PythonRobotics/PathPlanning/FrenetOptimalTrajectory/cartesian_frenet_converter.py
#             (https://github.com/AtsushiSakai/PythonRobotics)
# Author    : Wang Zheng (@Aglargil) / License : MIT
# Reference : Werling et al., "Optimal Trajectory Generation for Dynamic Street
#             Scenarios in a Frenet Frame", ICRA 2010.

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


# ==================== 시각화 ====================
# 후보를 색칠할 때 쓰는 colormap (낮은 비용 = 초록, 높은 비용 = 빨강)
COST_CMAP = 'RdYlGn_r'


# ---------------------------------------------------------------------------
# Reference path : cubic spline 을 감싼 얇은 래퍼
#   - state(s) 한 번 호출로 변환에 필요한 (rx, ry, ryaw, rkappa, rdkappa) 를 모두 제공
#   - CubicSpline2D 에는 곡률 변화율(rdkappa)이 없어 유한차분으로 근사한다.
# ---------------------------------------------------------------------------
class ReferencePath:
    """waypoint 를 cubic spline 으로 보간한 기준 경로(reference path)."""

    def __init__(self, wx, wy, ds=0.1):
        self.csp = CubicSpline2D(wx, wy)
        self.length = self.csp.s[-1]
        # 도로를 그릴 때 쓸 조밀한 샘플 (한 번만 계산해 둔다)
        self.s = np.arange(0.0, self.length, ds)
        self.x, self.y, self.yaw, self.k = [], [], [], []
        for s in self.s:
            ix, iy = self.csp.calc_position(s)
            self.x.append(ix)
            self.y.append(iy)
            self.yaw.append(self.csp.calc_yaw(s))
            self.k.append(self.csp.calc_curvature(s))
        self.x, self.y = np.array(self.x), np.array(self.y)
        self.yaw, self.k = np.array(self.yaw), np.array(self.k)

    def state(self, s):
        """기준 경로 위 경로 거리 s 지점의 (rx, ry, ryaw, rkappa, rdkappa)."""
        s = float(np.clip(s, 0.0, self.length - 1e-3))
        rx, ry = self.csp.calc_position(s)
        ryaw = self.csp.calc_yaw(s)
        rk = self.csp.calc_curvature(s)
        # 곡률 변화율은 유한차분으로 근사
        h = 0.1
        s2 = min(s + h, self.length - 1e-3)
        rk2 = self.csp.calc_curvature(s2)
        rdk = (rk2 - rk) / (s2 - s) if s2 > s else 0.0
        return rx, ry, ryaw, rk, rdk


# ---------------------------------------------------------------------------
# Frenet -> Cartesian 변환 : 위 FrenetConverter 사용
#   하나의 (s, d) 후보 곡선을 도로 위 (x, y, yaw, kappa, v) 로 옮긴다.
# ---------------------------------------------------------------------------
def FrenetPathToCartesian(ref, s_arr, d, d_d, d_dd, s_d, s_dd):
    """Frenet 곡선 (s, d(s)) 를 Cartesian 으로 변환한다.

    ref           : ReferencePath
    s_arr         : (M,) 경로 거리 s 배열
    d, d_d, d_dd  : (M,) 횡방향 offset 과 s 에 대한 1, 2차 미분
    s_d, s_dd     : 종방향 속도 s'(t), 가속도 s''(t)  (CV 이면 s_d=v, s_dd=0)
    반환          : dict(x, y, yaw, kappa, v)  (모두 (M,) 배열)
    """
    xs, ys, yaws, ks, vs = [], [], [], [], []
    for i, s in enumerate(s_arr):
        rx, ry, ryaw, rk, rdk = ref.state(s)
        x, y, theta, kappa, v, _a = FrenetConverter.frenet_to_cartesian(
            s, rx, ry, ryaw, rk, rdk,
            [s, s_d, s_dd], [d[i], d_d[i], d_dd[i]])
        xs.append(x); ys.append(y); yaws.append(theta)
        ks.append(kappa); vs.append(v)
    return {'x': np.array(xs), 'y': np.array(ys), 'yaw': np.array(yaws),
            'kappa': np.array(ks), 'v': np.array(vs)}


# ---------------------------------------------------------------------------
# 공통 배경 : 기준 경로 + 장애물 + 출발 차량
# ---------------------------------------------------------------------------
def DrawRoad(ax, ref, obstacles, robot_radius, start=None):
    """모든 Cartesian 시각화의 공통 배경(도로 + 장애물)을 그린다.

    start : (x, y) 출발 위치 표시 (없으면 생략)
    """
    ax.plot(ref.x, ref.y, '--', color='0.55', lw=1.8, zorder=1,
            label='reference path')
    for ox, oy in obstacles:
        ax.add_patch(Circle((ox, oy), robot_radius, fc='#f3b0b0', ec='crimson',
                            lw=1.4, alpha=0.85, zorder=2))
    # 장애물 중심 점
    if len(obstacles):
        obs = np.asarray(obstacles)
        ax.plot(obs[:, 0], obs[:, 1], 'x', color='crimson', ms=7, mew=2,
                zorder=3, label='obstacle')
    if start is not None:
        ax.plot(start[0], start[1], 'o', color='tab:blue', ms=11, zorder=5,
                label='start')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.grid(True, alpha=0.25)


def ShowRoad(ref, obstacles, robot_radius, start=None, title='Scenario: road + obstacles'):
    """DrawRoad 를 단독 figure 로 출력 (섹션 0 미리보기)."""
    fig, ax = plt.subplots(figsize=(11, 4.6))
    DrawRoad(ax, ref, obstacles, robot_radius, start=start)
    ax.set_title(title)
    # 축이 납작해 세로형 범례는 도로와 겹침 -> 가로 한 줄로 왼쪽 아래 빈 공간에 배치
    ax.legend(loc='lower left', ncol=3, fontsize=9)
    plt.tight_layout(); plt.show()


# ---------------------------------------------------------------------------
# 1. Frenet frame 개념 : 한 점의 (s, d) 분해
# ---------------------------------------------------------------------------
def PlotFrenetFrameConcept(ref, s_query, d_query):
    """같은 한 점을 Cartesian(굽은 경로)과 Frenet(s,d)(곧게 편 경로)로 나란히 비교한다.

    두 패널에서 색을 맞춰(주황 s, 초록 d, 파랑 vehicle, 검정 reference point)
    "굽은 경로를 곧게 펴면 (s, d) 가 그대로 가로와 세로 좌표가 된다"를 보인다.
    """
    rx, ry, ryaw, _rk, _rdk = ref.state(s_query)
    # 접선(s 방향) 단위벡터와 법선(d 방향, 왼쪽 +) 단위벡터
    tx, ty = np.cos(ryaw), np.sin(ryaw)
    nx, ny = -np.sin(ryaw), np.cos(ryaw)
    # 차량 위치 = 기준점에서 법선 방향으로 d_query 만큼
    px, py = rx + nx * d_query, ry + ny * d_query
    s_view = ref.s <= s_query + 15.0      # 출발점 s=0 부터 query 조금 너머까지

    fig, (axc, axf) = plt.subplots(1, 2, figsize=(13, 5.4))
    fig.suptitle(f'same point = (s, d)    here  s={s_query:.0f} m,  d={d_query:+.1f} m',
                 fontsize=13, y=1.02)

    # ---------- (a) Cartesian : 굽은 경로 그대로 ----------
    axc.plot(ref.x[s_view], ref.y[s_view], '-', color='0.45', lw=2.2, zorder=1,
             label='reference path')
    mask = ref.s <= s_query
    axc.plot(ref.x[mask], ref.y[mask], '-', color='tab:orange', lw=4, alpha=0.6,
             zorder=2, label='s : arc length')
    axc.annotate('', xy=(rx + tx * 5, ry + ty * 5), xytext=(rx, ry),
                 arrowprops=dict(arrowstyle='-|>', color='tab:orange', lw=2))
    axc.text(rx + tx * 5 + 0.7, ry + ty * 5 + 0.3, r'$s$ (tangent)', color='tab:orange',
             fontsize=11, fontweight='bold', ha='left', va='bottom')
    axc.annotate('', xy=(px, py), xytext=(rx, ry),
                 arrowprops=dict(arrowstyle='-|>', color='tab:green', lw=2.4))
    axc.text(rx - 0.5, (ry + py) / 2, r'$d$ (lateral offset)', color='tab:green',
             fontsize=11, fontweight='bold', ha='right', va='center')
    axc.plot(rx, ry, 'o', color='0.2', ms=7, zorder=5)
    axc.text(rx, ry - 1.6, 'reference point', ha='center', va='top',
             fontsize=9, color='0.2')
    axc.plot(px, py, 's', color='tab:blue', ms=11, zorder=6)
    axc.text(px, py + 0.8, 'vehicle', color='tab:blue', fontsize=10,
             fontweight='bold', ha='center', va='bottom')
    xs = ref.x[s_view]; ys = ref.y[s_view]
    axc.set_xlim(min(xs.min(), px) - 3, max(xs.max(), px) + 8)
    axc.set_ylim(min(ys.min(), py) - 3, max(ys.max(), py) + 4)
    axc.set_aspect('equal'); axc.grid(True, alpha=0.25)
    axc.set_xlabel('x [m]'); axc.set_ylabel('y [m]')
    axc.set_title('(a) Cartesian: curved path')
    axc.legend(loc='lower right', fontsize=9)

    # ---------- (b) Frenet : 경로를 곧게 편 (s, d) 평면 ----------
    s_max = float(ref.s[s_view][-1])
    axf.axhline(0, color='0.45', lw=2.2, zorder=1, label='reference path (d=0)')
    axf.plot([0, s_query], [0, 0], '-', color='tab:orange', lw=4, alpha=0.6,
             zorder=2, label='s : arc length')
    axf.annotate('', xy=(s_query + 5, 0), xytext=(s_query, 0),
                 arrowprops=dict(arrowstyle='-|>', color='tab:orange', lw=2))
    axf.text(s_query + 5 + 0.8, 0.3, r'$s$ (tangent)', color='tab:orange',
             fontsize=11, fontweight='bold', ha='left', va='bottom')
    axf.annotate('', xy=(s_query, d_query), xytext=(s_query, 0),
                 arrowprops=dict(arrowstyle='-|>', color='tab:green', lw=2.4))
    axf.text(s_query - 0.5, d_query / 2, r'$d$ (lateral offset)', color='tab:green',
             fontsize=11, fontweight='bold', ha='right', va='center')
    axf.plot(s_query, 0, 'o', color='0.2', ms=7, zorder=5)
    axf.text(s_query, -0.45, 'reference point', ha='center', va='top',
             fontsize=9, color='0.2')
    axf.plot(s_query, d_query, 's', color='tab:blue', ms=11, zorder=6)
    axf.text(s_query, d_query + 0.5, 'vehicle', color='tab:blue', fontsize=10,
             fontweight='bold', ha='center', va='bottom')
    axf.set_xlim(-2, s_max + 3)
    axf.set_ylim(-3, max(d_query + 2.0, 4.0))
    axf.grid(True, alpha=0.25)
    axf.set_xlabel('s [m]  (arc length)'); axf.set_ylabel('d [m]  (lateral offset)')
    axf.set_title('(b) Frenet (s, d): path straightened')
    axf.legend(loc='upper right', fontsize=9)

    plt.tight_layout(rect=[0, 0, 1, 0.95]); plt.show()


# ---------------------------------------------------------------------------
# 2. 종방향 profile (CV) 고정
# ---------------------------------------------------------------------------
def PlotLongitudinalProfile(s0, v, T, dt=0.1):
    """고정된 종방향 프로파일 s(t)=s0+v t (등속 CV) 를 그린다."""
    t = np.arange(0.0, T + dt, dt)
    s_cv = s0 + v * t
    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.plot(t, s_cv, color='tab:blue', lw=2.4, label=r'CV : $s(t)=s_0+vt$  (used)')
    ax.set_xlabel('time t [s]'); ax.set_ylabel('arc length s [m]')
    ax.set_title('Fixed longitudinal profile (constant velocity)')
    ax.grid(True, alpha=0.3); ax.legend(loc='best', fontsize=9)
    plt.tight_layout(); plt.show()


# ---------------------------------------------------------------------------
# 3. 횡방향 후보 : Frenet 좌표(d vs s)
# ---------------------------------------------------------------------------
def PlotLateralCandidatesFrenet(candidates):
    """여러 d_i 후보의 d(s) 곡선을 Frenet 평면에서 부채꼴로 그린다."""
    fig, ax = plt.subplots(figsize=(9, 4.6))
    for cand in candidates:
        ax.plot(cand.s, cand.d, '-', color='tab:gray', lw=1.2, alpha=0.8)
        ax.plot(cand.s[-1], cand.d[-1], 'o', color='tab:orange', ms=4)
    ax.axhline(0, color='0.4', lw=1.2, ls='--', label='reference path (d=0)')
    ax.set_xlabel('arc length s [m]'); ax.set_ylabel('lateral offset d [m]')
    ax.set_title(f'Lateral candidates in Frenet frame  ({len(candidates)} quintic curves)')
    ax.grid(True, alpha=0.3); ax.legend(loc='best', fontsize=9)
    plt.tight_layout(); plt.show()


# ---------------------------------------------------------------------------
# 4. 후보를 도로 위(Cartesian)에 그리기
# ---------------------------------------------------------------------------
def PlotCandidatesCartesian(ref, obstacles, robot_radius, candidates, start=None):
    """변환된 후보들을 도로 위에 부채꼴로 그린다(슬라이드의 path candidates)."""
    fig, ax = plt.subplots(figsize=(11, 5.0))
    DrawRoad(ax, ref, obstacles, robot_radius, start=start)
    for cand in candidates:
        ax.plot(cand.x, cand.y, '-', color='tab:gray', lw=1.1, alpha=0.85, zorder=4)
    ax.set_title('Trajectory candidates on the road (Frenet -> Cartesian)')
    ax.legend(loc='best', fontsize=9)
    plt.tight_layout(); plt.show()


# ---------------------------------------------------------------------------
# 5. 후보를 비용으로 색칠
# ---------------------------------------------------------------------------
def PlotCandidatesCost(ref, obstacles, robot_radius, candidates, start=None):
    """후보들을 cost 로 색칠(낮을수록 초록)."""
    costs = np.array([c.cost for c in candidates])
    vmin, vmax = costs.min(), costs.max()
    cmap = plt.get_cmap(COST_CMAP)
    norm = plt.Normalize(vmin, vmax)

    fig, ax = plt.subplots(figsize=(11, 5.0))
    DrawRoad(ax, ref, obstacles, robot_radius, start=start)
    # 비용 큰 것 -> 작은 것 순서로 그려, 좋은(초록) 후보가 위에 보이게
    for cand in sorted(candidates, key=lambda c: -c.cost):
        ax.plot(cand.x, cand.y, '-', color=cmap(norm(cand.cost)), lw=1.8,
                alpha=0.9, zorder=4)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm); sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.035, pad=0.02)
    cbar.set_label('cost (lower = better)')
    ax.set_title('Candidates colored by cost')
    ax.legend(loc='best', fontsize=9)
    plt.tight_layout(); plt.show()


# ---------------------------------------------------------------------------
# 6. 유효/무효(충돌) 표시
# ---------------------------------------------------------------------------
def PlotValidity(ref, obstacles, robot_radius, candidates, start=None):
    """충돌/제약 위반 후보는 빨강, 통과 후보는 초록으로 그린다."""
    fig, ax = plt.subplots(figsize=(11, 5.0))
    DrawRoad(ax, ref, obstacles, robot_radius, start=start)
    n_ok = 0
    for cand in candidates:
        if cand.valid:
            ax.plot(cand.x, cand.y, '-', color='tab:green', lw=1.6, alpha=0.9, zorder=4)
            n_ok += 1
        else:
            ax.plot(cand.x, cand.y, '-', color='tab:red', lw=1.0, alpha=0.45, zorder=3)
    ax.set_title(f'Valid (green, n={n_ok}) vs invalid (red, n={len(candidates) - n_ok})')
    ax.legend(loc='best', fontsize=9)
    plt.tight_layout(); plt.show()


# ---------------------------------------------------------------------------
# 7. best candidate 강조
# ---------------------------------------------------------------------------
def PlotBest(ref, obstacles, robot_radius, candidates, best, start=None):
    """유효 후보(연한 초록) 위에 최소비용 best 경로를 굵게 강조."""
    fig, ax = plt.subplots(figsize=(11, 5.0))
    DrawRoad(ax, ref, obstacles, robot_radius, start=start)
    for cand in candidates:
        if cand.valid:
            ax.plot(cand.x, cand.y, '-', color='tab:green', lw=1.0, alpha=0.30, zorder=3)
    ax.plot(best.x, best.y, '-', color='tab:blue', lw=3.2, zorder=6,
            label=f'best (cost={best.cost:.2f}, d_end={best.di:+.1f} m)')
    ax.set_title('Selected best candidate (minimum cost, collision-free)')
    ax.legend(loc='best', fontsize=9)
    plt.tight_layout(); plt.show()
