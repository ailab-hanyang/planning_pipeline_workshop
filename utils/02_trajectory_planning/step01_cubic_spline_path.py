"""
Visualization helpers for step01_cubic_spline_path notebook.

긴 시각화 코드를 노트북에서 분리. 노트북은 각 함수를 한 줄로 호출.
plot 제목/축라벨은 영어(matplotlib 한글 폰트 깨짐 방지).

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from utils.cubic_spline import CubicSpline1D

SEGMENT_COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']


def plot_straight_line_limit(x, y):
    """직선 연결의 한계: 경로(왼쪽)와 코너마다 점프하는 방향각(오른쪽)."""
    seg_heading, s = [], [0.0]
    for i in range(len(x) - 1):
        dx, dy = x[i + 1] - x[i], y[i + 1] - y[i]
        seg_heading.append(np.degrees(np.arctan2(dy, dx)))
        s.append(s[-1] + np.hypot(dx, dy))

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 5),
                                   gridspec_kw={'width_ratios': [1.25, 1.0]})

    axL.plot(x, y, '-', color='gray', lw=2, label='Straight lines')
    axL.plot(x, y, 'o', color='tab:orange', ms=11, zorder=3, label='Points')
    for i in range(1, len(x) - 1):
        axL.plot(x[i], y[i], 'o', mfc='none', mec='red', mew=2.2, ms=20, zorder=4)
        axL.annotate(f'{i}', xy=(x[i], y[i]), xytext=(0, 14), textcoords='offset points',
                     ha='center', color='red', fontsize=10, fontweight='bold')
    axL.set_xlim(min(x) - 1, max(x) + 1); axL.set_ylim(min(y) - 2, max(y) + 2.5)
    axL.grid(True, alpha=0.3); axL.legend(loc='lower right')
    axL.set_title('Path: straight lines  (red = corners)')
    axL.set_xlabel('x'); axL.set_ylabel('y')

    for i in range(len(seg_heading)):
        axR.hlines(seg_heading[i], s[i], s[i + 1], color='tab:blue', lw=3, zorder=2)
        axR.plot(s[i], seg_heading[i], 'o', color='tab:blue', ms=5, zorder=3)
    for i in range(1, len(seg_heading)):
        jump = seg_heading[i] - seg_heading[i - 1]
        axR.vlines(s[i], seg_heading[i - 1], seg_heading[i], color='red', ls='--', lw=1.8)
        axR.annotate(f'{jump:+.0f}°', xy=(s[i], (seg_heading[i - 1] + seg_heading[i]) / 2),
                     xytext=(5, 0), textcoords='offset points', color='red', fontsize=9, va='center')
        axR.annotate(f'{i}', xy=(s[i], max(seg_heading) + 6), ha='center',
                     color='red', fontsize=10, fontweight='bold')
    axR.axhline(0, color='gray', lw=0.8, alpha=0.5)
    axR.set_ylim(min(seg_heading) - 10, max(seg_heading) + 14)
    axR.grid(True, alpha=0.3)
    axR.set_title('Heading jumps at every corner (discontinuous)')
    axR.set_xlabel('distance along path'); axR.set_ylabel('heading (deg)')

    plt.tight_layout(); plt.show()


def plot_segment_definition(x, y):
    """정의 그림: 조각별 색 곡선 + S0/S1 라벨(왼쪽), 이음매 연속 조건(오른쪽)."""
    sp = CubicSpline1D(x, y)

    fig, (ax, ax_text) = plt.subplots(1, 2, figsize=(13, 4.5),
                                      gridspec_kw={'width_ratios': [2.3, 1.0]})

    ax.plot(x, y, '-', color='lightgray', lw=1.5, zorder=1)
    for i in range(len(x) - 1):
        seg = np.linspace(x[i], x[i + 1], 50)
        ax.plot(seg, [sp.calc_position(t) for t in seg],
                color=SEGMENT_COLORS[i % len(SEGMENT_COLORS)], lw=3,
                solid_capstyle='round', zorder=2)
    ax.plot(x, y, 'o', mfc='white', mec='black', mew=1.5, ms=9, zorder=3)

    ax.axvline(x[1], color='gray', ls=':', lw=1.0, alpha=0.7)
    ax.annotate(r'$x_1$', xy=(x[1], -0.6), ha='center', color='gray', fontsize=12)

    ax.annotate(r'$S_0(\Delta)=a_0+b_0\Delta+c_0\Delta^2+d_0\Delta^3$',
                xy=(1.0, sp.calc_position(1.0)), xytext=(-1.0, 7.8), ha='left',
                color='tab:blue', fontsize=10,
                arrowprops=dict(arrowstyle='->', color='tab:blue'))
    ax.annotate(r'$S_1(\Delta)=a_1+b_1\Delta+c_1\Delta^2+d_1\Delta^3$',
                xy=(3.0, sp.calc_position(3.0)), xytext=(5.0, 8.4), ha='left',
                color='tab:orange', fontsize=10,
                arrowprops=dict(arrowstyle='->', color='tab:orange'))

    ax.set_xlim(-1.5, 13); ax.set_ylim(-1.5, 9.5)
    ax.set_title('Each segment is one cubic; pieces join smoothly')
    ax.set_xlabel('x'); ax.set_ylabel('y')
    ax.grid(True, alpha=0.3)

    ax_text.axis('off')
    conditions = [
        (r"$S_0(x_1)=S_1(x_1)$",     r"$(C^0)$  position"),
        (r"$S_0'(x_1)=S_1'(x_1)$",   r"$(C^1)$  slope"),
        (r"$S_0''(x_1)=S_1''(x_1)$", r"$(C^2)$  curvature"),
    ]
    for row, (equation, label) in enumerate(conditions):
        y_pos = 0.74 - row * 0.26
        ax_text.text(0.00, y_pos, equation, fontsize=15, transform=ax_text.transAxes)
        ax_text.text(0.55, y_pos, label, fontsize=12, color='dimgray', transform=ax_text.transAxes)

    plt.tight_layout(); plt.show()


def plot_slope_comparison(x, y, evaluate_cubic_slope):
    """경로를 따라 직선 기울기(점프)와 스플라인 기울기(연속) 비교.

    evaluate_cubic_slope : 학생이 채운 기울기 함수 (a, b, c, d, delta) -> slope
    """
    sp = CubicSpline1D(x, y)

    dense_x, straight_slope, spline_slope = [], [], []
    for i in range(len(x) - 1):
        line_slope = (y[i + 1] - y[i]) / (x[i + 1] - x[i])
        for t in np.linspace(x[i], x[i + 1], 50, endpoint=False):
            dense_x.append(t)
            straight_slope.append(line_slope)
            spline_slope.append(evaluate_cubic_slope(sp.a[i], sp.b[i], sp.c[i], sp.d[i], t - x[i]))

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(dense_x, straight_slope, color='gray', lw=2, label='Straight lines: slope jumps')
    ax.plot(dense_x, spline_slope, color='tab:blue', lw=2.5, label='Spline: slope is continuous')
    for knot_x in x[1:-1]:
        ax.axvline(knot_x, color='red', ls='--', lw=0.8, alpha=0.5)
    ax.grid(True, alpha=0.3); ax.legend(loc='upper right')
    ax.set_title('Slope (dy/dx) along the path  (red lines = joints)')
    ax.set_xlabel('x'); ax.set_ylabel('slope dy/dx')
    plt.tight_layout(); plt.show()


def plot_cubic_spline(x, y, evaluate_cubic_segment):
    """완성된 cubic spline을 조각별 다른 색으로 시각화.

    evaluate_cubic_segment : 학생이 채운 위치 함수 (a, b, c, d, delta) -> position
    """
    sp = CubicSpline1D(x, y)

    fig, ax = plt.subplots(figsize=(9, 5))
    for i in range(len(x) - 1):
        seg = np.linspace(x[i], x[i + 1], 50)
        positions = [evaluate_cubic_segment(sp.a[i], sp.b[i], sp.c[i], sp.d[i], t - x[i]) for t in seg]
        ax.plot(seg, positions, lw=4, solid_capstyle='round', label=f'S{i}')
    ax.plot(x, y, 'o', color='black', ms=9, zorder=5, label='Points')
    ax.grid(True, alpha=0.3); ax.legend(loc='lower right', ncol=2, fontsize=8)
    ax.set_title('Cubic spline (each segment a different color)')
    ax.set_xlabel('x'); ax.set_ylabel('y')
    plt.tight_layout(); plt.show()


def interactive_spline(x, y, evaluate_cubic_segment):
    """점 하나를 슬라이더로 움직이면 spline이 다시 그려지는 interactive 위젯.

    evaluate_cubic_segment : 학생이 채운 위치 함수 (a, b, c, d, delta) -> position
    """
    import ipywidgets as widgets

    base_x, base_y = list(x), list(y)

    def show_spline(point=3, y_value=4.0):
        ys = list(base_y)
        ys[point] = y_value
        sp = CubicSpline1D(base_x, ys)
        fig, ax = plt.subplots(figsize=(9, 5))
        for i in range(len(base_x) - 1):
            seg = np.linspace(base_x[i], base_x[i + 1], 40)
            positions = [evaluate_cubic_segment(sp.a[i], sp.b[i], sp.c[i], sp.d[i], t - base_x[i]) for t in seg]
            ax.plot(seg, positions, lw=3, solid_capstyle='round')
        ax.plot(base_x, ys, 'o', color='black', ms=8, zorder=4)
        ax.plot(base_x[point], y_value, 'o', mfc='none', mec='red', mew=2.5, ms=16, zorder=5)
        ax.set_xlim(min(base_x) - 1, max(base_x) + 1); ax.set_ylim(-3, 11)
        ax.grid(True, alpha=0.3)
        ax.set_title(f'Move point {point} -> spline updates')
        ax.set_xlabel('x'); ax.set_ylabel('y')
        plt.show()

    return widgets.interact(
        show_spline,
        point=widgets.IntSlider(min=1, max=5, step=1, value=3, description='point'),
        y_value=widgets.FloatSlider(min=-3, max=11, step=0.25, value=base_y[3], description='y'),
    )
