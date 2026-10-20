"""
Visualization helpers for step02_longitudinal_motion_planning notebook.

긴 시각화 코드를 노트북에서 분리. 노트북은 각 함수를 한 줄로 호출.
경로(x, y) 위에 슬라이드와 같은 장면 요소(제한속도 표지판/30 구간/방지턱/정지선/자차)를
DrawScene 하나로 그려 모든 시각화의 배경으로 재사용한다.
plot 제목/축라벨은 영어(matplotlib 한글 폰트 깨짐 방지). patches 만 사용(외부 이미지 없음).

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, Wedge, FancyBboxPatch
from matplotlib.transforms import Affine2D

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

MS2KPH = 3.6                       # m/s -> km/h
SPEED_CMAP = 'RdYlGn'              # 낮은 속도 = 빨강, 높은 속도 = 초록
# 모든 프로파일 그래프의 가로축: 경로를 따라 잰 거리(arc length s). 명시적으로 설명.
S_XLABEL = 'arc length s [m]  (distance along the path)'


# ---------------------------------------------------------------------------
# scene 헬퍼
# ---------------------------------------------------------------------------
def IndexAtS(scene, s_query):
    """arc-length s_query 에 가장 가까운 샘플 인덱스."""
    return int(np.argmin(np.abs(scene['s'] - s_query)))


def _pose_at_s(scene, s_query):
    """s_query 위치의 (x, y, yaw)."""
    i = IndexAtS(scene, s_query)
    return scene['x'][i], scene['y'][i], scene['yaw'][i]


def _perp(yaw):
    """경로 접선각 yaw 에 대한 왼쪽(법선) 단위벡터."""
    return -np.sin(yaw), np.cos(yaw)


# ---------------------------------------------------------------------------
# 아이콘 (matplotlib patches 만 사용)
# ---------------------------------------------------------------------------
def _car_patches(color):
    """차량 아이콘을 로컬 좌표(중심 원점, 머리 방향 +x)에서 정의한 patch 리스트."""
    length, width = 8.0, 4.0
    patches = [
        FancyBboxPatch((-length / 2, -width / 2), length, width,
                       boxstyle='round,pad=0,rounding_size=1.2',
                       fc=color, ec='black', lw=1.4, zorder=6),
        # 캐빈(앞 유리) — 머리 방향을 알 수 있게 앞쪽에 밝은 사각형
        Rectangle((0.3, -width * 0.34), length * 0.26, width * 0.68,
                  fc='#cfe8ff', ec='black', lw=0.8, zorder=7),
    ]
    # 바퀴 4개
    wheel_w, wheel_h = 1.6, 0.7
    for wx in (-length * 0.3, length * 0.3):
        for wy in (-width / 2 - wheel_h * 0.4, width / 2 - wheel_h * 0.6):
            patches.append(Rectangle((wx - wheel_w / 2, wy), wheel_w, wheel_h,
                                     fc='black', ec='black', zorder=5))
    return patches


def _place_patches(patches, ax, x, y, yaw):
    """로컬 좌표 patch 들을 (x, y, yaw) 로 회전·이동시켜 배치."""
    transform = Affine2D().rotate(yaw).translate(x, y) + ax.transData
    for patch in patches:
        patch.set_transform(transform)


def _add_car(ax, x, y, yaw, color, label=None):
    patches = _car_patches(color)
    _place_patches(patches, ax, x, y, yaw)
    for patch in patches:
        ax.add_patch(patch)
    if label:
        nx, ny = _perp(yaw)
        ax.annotate(label, xy=(x + nx * 7, y + ny * 7), ha='center', va='center',
                    fontsize=9, fontweight='bold', color=color,
                    bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=color, alpha=0.9),
                    zorder=8)
    return patches


def _add_speed_sign(ax, x, y, yaw, limit_text, offset=9.0):
    """제한속도 표지판: 흰 배경 + 빨간 링 + 가운데 숫자. 경로 위쪽에."""
    nx, ny = _perp(yaw)
    cx, cy = x + nx * offset, y + ny * offset
    ax.add_patch(Circle((cx, cy), 4.6, fc='white', ec='red', lw=3.2, zorder=7))
    ax.text(cx, cy, limit_text, ha='center', va='center',
            fontsize=11, fontweight='bold', color='black', zorder=8)


def _add_bump(ax, x, y, yaw):
    """방지턱: 경로 위 주황색 반원(Wedge, 채움) + 옆에 경고 삼각형."""
    ax.add_patch(Wedge((x, y), 3.4, np.degrees(yaw), np.degrees(yaw) + 180,
                       fc='orange', ec='saddlebrown', lw=1.4, zorder=6))
    nx, ny = _perp(yaw)
    tx, ty = x + nx * 8.5, y + ny * 8.5
    triangle = Polygon([(tx, ty + 3.0), (tx - 2.7, ty - 2.2), (tx + 2.7, ty - 2.2)],
                       closed=True, fc='gold', ec='red', lw=2.0, zorder=7)
    ax.add_patch(triangle)
    ax.text(tx, ty - 0.4, '!', ha='center', va='center',
            fontsize=11, fontweight='bold', color='red', zorder=8)


def _add_stop(ax, x, y, yaw, half_width=6.0):
    """정지선(경로와 수직인 굵은 빨간 선분) + STOP 라벨."""
    nx, ny = _perp(yaw)
    ax.plot([x - nx * half_width, x + nx * half_width],
            [y - ny * half_width, y + ny * half_width],
            color='red', lw=5, solid_capstyle='butt', zorder=5)
    ax.annotate('STOP', xy=(x + nx * 9, y + ny * 9), ha='center', va='center',
                fontsize=10, fontweight='bold', color='red',
                bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='red'), zorder=8)


# ---------------------------------------------------------------------------
# DrawScene : 모든 경로 시각화의 공통 배경
# ---------------------------------------------------------------------------
def DrawScene(ax, scene, v_kph=None, show_cbar=True, show_icons=True):
    """경로 위에 시나리오 요소를 그린다. v_kph 가 주어지면 경로 점을 속도로 색칠.

    scene : dict ( s, x, y, yaw + 시나리오 파라미터 )
    v_kph : (N,) 속도[km/h] 배열 또는 None
    반환  : scatter 핸들(없으면 None) — colorbar 용
    """
    x, y, yaw = scene['x'], scene['y'], scene['yaw']
    scatter = None

    ax.plot(x, y, '-', color='0.55', lw=2.2, zorder=1)

    # 30 km/h 제한속도 구간을 도로 위에 색으로 표시 (구간이 어디인지 한눈에)
    in_zone = (scene['s'] >= scene['limit30_start']) & (scene['s'] <= scene['limit30_end'])
    ax.plot(np.asarray(x)[in_zone], np.asarray(y)[in_zone], '-', color='royalblue',
            lw=12, alpha=0.30, solid_capstyle='round', zorder=1.2)
    mzx, mzy, mzyaw = _pose_at_s(scene, 0.5 * (scene['limit30_start'] + scene['limit30_end']))
    nzx, nzy = _perp(mzyaw)
    # 이후 프로파일 그래프의 가로축이 s 라서, 장면에도 s 값을 병기해 좌표를 잇는다
    ax.annotate(f"30 zone\ns={scene['limit30_start']:.0f}~{scene['limit30_end']:.0f}",
                xy=(mzx - nzx * 8, mzy - nzy * 8), ha='center', va='center',
                fontsize=8, fontweight='bold', color='royalblue', zorder=8)

    if v_kph is not None:
        scatter = ax.scatter(x, y, c=v_kph, cmap=SPEED_CMAP, vmin=0,
                             vmax=scene['v_limit_kph'], s=22, zorder=2)
        if show_cbar:
            cbar = ax.figure.colorbar(scatter, ax=ax, fraction=0.035, pad=0.02)
            cbar.set_label('speed [km/h]')

    if show_icons:
        # 제한속도 표지판 60 (출발부)
        sx, sy, syaw = _pose_at_s(scene, scene['s'][-1] * 0.05)
        _add_speed_sign(ax, sx, sy, syaw, str(int(scene['v_limit_kph'])))
        # 제한속도 표지판 30 (30 km/h 구간 시작부)
        zx, zy, zyaw = _pose_at_s(scene, scene['limit30_start'])
        _add_speed_sign(ax, zx, zy, zyaw, str(int(scene['limit30_kph'])))
        # 방지턱 (+ s 좌표 병기)
        bx, by, byaw = _pose_at_s(scene, 0.5 * (scene['bump_start'] + scene['bump_end']))
        _add_bump(ax, bx, by, byaw)
        nbx, nby = _perp(byaw)
        ax.annotate(f"s={scene['bump_start']:.0f}~{scene['bump_end']:.0f}",
                    xy=(bx - nbx * 6, by - nby * 6), ha='center', va='center',
                    fontsize=8, color='saddlebrown', zorder=8)
        # 정지선 (+ s 좌표 병기)
        tx, ty, tyaw = _pose_at_s(scene, scene['s_stop'])
        _add_stop(ax, tx, ty, tyaw)
        ntx, nty = _perp(tyaw)
        ax.annotate(f"s={scene['s_stop']:.0f}", xy=(tx - ntx * 6, ty - nty * 6),
                    ha='center', va='center', fontsize=8, color='red', zorder=8)
        # 자차(ego) — 시작 위치
        ex, ey, eyaw = _pose_at_s(scene, 0.0)
        _add_car(ax, ex, ey, eyaw, '#3b6fb0', label='ego')

    ax.set_aspect('equal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.margins(0.08)
    ax.grid(True, alpha=0.25)
    return scatter


def ShowScene(scene, v_kph=None, title=None, figsize=(12, 4.2)):
    """DrawScene 를 단독 figure 로 출력 (경로 점을 속도로 색칠)."""
    fig, ax = plt.subplots(figsize=figsize)
    DrawScene(ax, scene, v_kph=v_kph)
    if title:
        ax.set_title(title)
    plt.tight_layout(); plt.show()


# ---------------------------------------------------------------------------
# scene(상단) + metric(하단) 을 세로로 쌓는 공통 레이아웃
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# 섹션별 그래프 (각 plot 은 경로 거리 s [m] 를 가로축으로 하는 단독 그래프)
# ---------------------------------------------------------------------------
def _mark_events(ax, scene):
    """s 축 그래프에 급커브 / 30 km/h 구간 / 방지턱 / 정지선 위치 표시."""
    # 급커브 구간: 곡률이 커서 v_kappa 가 제한속도 아래로 떨어지는 곳 (yaw 변화율로 추정).
    # 속도 프로파일의 골이 '어느 커브 때문'인지 배경만 봐도 알 수 있게 한다.
    s = np.asarray(scene['s'])
    kappa_scene = np.abs(np.gradient(np.unwrap(np.asarray(scene['yaw'])), s))
    v_limit_ms = scene['v_limit_kph'] / MS2KPH
    in_curve = kappa_scene > scene.get('a_lat_max', 2.0) / v_limit_ms ** 2
    edges = np.flatnonzero(np.diff(in_curve.astype(int)))
    bounds = np.r_[0, edges + 1, len(s)]
    curve_label = 'sharp curve'
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        if in_curve[lo]:
            ax.axvspan(s[lo], s[hi - 1], color='tab:purple', alpha=0.10, label=curve_label)
            curve_label = None
    ax.axvspan(scene['limit30_start'], scene['limit30_end'], color='tab:blue', alpha=0.10,
               label='30 km/h zone')
    ax.axvspan(scene['bump_start'], scene['bump_end'], color='orange', alpha=0.20,
               label='bump zone')
    ax.axvline(scene['s_stop'], color='red', ls='--', lw=1.5, label='stop line')


def PlotCurvature(scene, kappa):
    """곡률 κ vs 경로 거리 s. 곡선 구간(κ 큰 곳)이 한눈에 보이게."""
    s = scene['s']
    fig, ax = plt.subplots(figsize=(11, 3.8))
    ax.plot(s, kappa, color='tab:purple', lw=2)
    ax.fill_between(s, kappa, alpha=0.15, color='tab:purple')
    ax.axhline(0, color='gray', lw=0.8)
    ax.set_xlabel(S_XLABEL); ax.set_ylabel(r'curvature $\kappa$ [1/m]')
    ax.set_title('Curvature along the path (large |kappa| = sharp curve)')
    ax.set_xlim(0, s[-1])
    ax.grid(True, alpha=0.3)
    plt.tight_layout(); plt.show()


def _draw_profiles(ax, scene, named, ylim=None):
    """주어진 axis 에 여러 속도 프로파일(s vs km/h)을 그린다.

    named : list of dict { 'v': (N,) m/s, 'label', 'color', 'ls'(opt), 'lw'(opt), 'alpha'(opt) }
    """
    s = scene['s']
    _mark_events(ax, scene)
    for item in named:
        v_kph = np.array(item['v']) * MS2KPH
        v_kph = np.where(np.isfinite(v_kph), v_kph, np.nan)
        ax.plot(s, v_kph, label=item['label'], color=item['color'],
                ls=item.get('ls', '-'), lw=item.get('lw', 2.0), alpha=item.get('alpha', 1.0))
    ax.axhline(scene['v_limit_kph'], color='gray', lw=0.8, ls=':')
    ax.set_xlabel(S_XLABEL); ax.set_ylabel('speed [km/h]')
    ax.set_ylim(ylim if ylim else (-3, scene['v_limit_kph'] * 1.15))
    ax.set_xlim(0, s[-1])
    ax.grid(True, alpha=0.3)
    # 범례는 왼쪽 아래(항상 빈 공간)에 -- 오른쪽 위는 방지턱/정지선 구간을 가림
    ax.legend(loc='lower left', fontsize=8, ncol=2)


def PlotProfiles(scene, named, title, ylim=None, figsize=(11, 4.0)):
    """여러 속도 프로파일을 경로 거리 s 기준으로 겹쳐 그린다(km/h)."""
    fig, ax = plt.subplots(figsize=figsize)
    _draw_profiles(ax, scene, named, ylim)
    ax.set_title(title)
    plt.tight_layout(); plt.show()


def PlotSmoothing(scene, before, after, before_label, after_label, title):
    """smoothing 전/후 프로파일 비교 (km/h)."""
    PlotProfiles(scene, [
        {'v': before, 'label': before_label, 'color': 'orange', 'ls': '--', 'lw': 1.6, 'alpha': 0.9},
        {'v': after,  'label': after_label,  'color': 'tab:blue', 'lw': 2.6},
    ], title)


def PlotValidation(scene, v_final, accel, a_min, a_max):
    """최종 속도 프로파일 + 가속도 프로파일(한계선 포함). 둘 다 경로 거리 s 기준."""
    s = scene['s']
    fig, (ax_v, ax_a) = plt.subplots(2, 1, figsize=(11, 5.6), sharex=True)

    _mark_events(ax_v, scene)
    # 곡선 아래 fill 은 30 zone 배경 밴드와 색이 겹쳐 밴드를 가리므로 넣지 않는다
    ax_v.plot(s, np.array(v_final) * MS2KPH, color='tab:blue', lw=2.6, label='final speed')
    ax_v.axhline(scene['v_limit_kph'], color='gray', lw=0.8, ls=':')
    ax_v.set_ylabel('speed [km/h]'); ax_v.set_ylim(-3, scene['v_limit_kph'] * 1.15)
    ax_v.set_title('Final speed profile')
    ax_v.grid(True, alpha=0.3); ax_v.legend(loc='lower left', fontsize=8, ncol=2)

    ax_a.plot(s, accel, color='tab:green', lw=2)
    ax_a.axhline(a_max, color='red', ls='--', lw=1.4, label=f'a_max = {a_max}')
    ax_a.axhline(a_min, color='red', ls='--', lw=1.4, label=f'a_min = {a_min}')
    ax_a.axhline(0, color='gray', lw=0.8)
    ax_a.fill_between(s, a_min, a_max, color='green', alpha=0.07)
    ax_a.set_xlabel(S_XLABEL); ax_a.set_ylabel(r'accel $a$ [m/s$^2$]')
    ax_a.set_title('Acceleration stays within [a_min, a_max]')
    ax_a.grid(True, alpha=0.3); ax_a.legend(loc='upper left', fontsize=8)
    ax_a.set_xlim(0, s[-1])
    plt.tight_layout(); plt.show()


def PlotTimeParameterization(t, s_t, v_t):
    """시간 적분 결과 v(t), s(t)."""
    fig, (ax_v, ax_s) = plt.subplots(2, 1, figsize=(10, 5.0), sharex=True)
    ax_v.plot(t, np.array(v_t) * MS2KPH, color='tab:blue', lw=2.4)
    ax_v.fill_between(t, np.array(v_t) * MS2KPH, alpha=0.12, color='tab:blue')
    ax_v.set_ylabel('speed [km/h]'); ax_v.set_title('Speed vs time  v(t)')
    ax_v.grid(True, alpha=0.3)
    ax_s.plot(t, s_t, color='tab:purple', lw=2.4)
    ax_s.set_xlabel('time t [s]'); ax_s.set_ylabel('arc length s [m]')
    ax_s.set_title('Distance vs time  s(t)')
    ax_s.grid(True, alpha=0.3)
    plt.tight_layout(); plt.show()


# ---------------------------------------------------------------------------
# 주행 애니메이션
# ---------------------------------------------------------------------------
def AnimateDrive(scene, t_frames, s_frames, v_frames_kph, interval=60):
    """v(t), s(t) 를 따라 자차가 경로를 주행하는 애니메이션.

    위: DrawScene 배경 + 움직이는 자차 / 아래: 속도 프로파일 + 현재 위치 커서.
    반환 : matplotlib FuncAnimation  (노트북에서 HTML(anim.to_jshtml()) 로 재생)
    """
    from matplotlib.animation import FuncAnimation

    s, x, y, yaw = scene['s'], scene['x'], scene['y'], scene['yaw']

    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(12, 7.2),
                                         gridspec_kw={'height_ratios': [1.7, 1]})

    # 위: 정적 장면(아이콘+표지판 등)을 한 번만 그림. 자차는 따로 움직임.
    DrawScene(ax_top, scene, v_kph=None, show_cbar=False)
    full_v_kph = np.interp(s, s_frames, v_frames_kph)
    ax_top.scatter(x, y, c=full_v_kph, cmap=SPEED_CMAP, vmin=0,
                   vmax=scene['v_limit_kph'], s=16, zorder=2)
    ax_top.set_title('Driving along the path (color = planned speed)')

    ego_patches = _car_patches('#3b6fb0')
    for patch in ego_patches:
        patch.set_zorder(9)
        ax_top.add_patch(patch)

    # 아래: 속도 프로파일 + 현재 위치 커서
    _mark_events(ax_bot, scene)
    ax_bot.plot(s, full_v_kph, color='tab:blue', lw=2.2)
    ax_bot.fill_between(s, full_v_kph, alpha=0.12, color='tab:blue')
    ax_bot.axhline(scene['v_limit_kph'], color='gray', lw=0.8, ls=':')
    ax_bot.set_xlabel(S_XLABEL); ax_bot.set_ylabel('speed [km/h]')
    ax_bot.set_ylim(-3, scene['v_limit_kph'] * 1.15)
    ax_bot.grid(True, alpha=0.3)
    cursor = ax_bot.axvline(s_frames[0], color='red', lw=2)
    speed_text = ax_bot.text(0.012, 0.86, '', transform=ax_bot.transAxes,
                             fontsize=12, fontweight='bold', color='red',
                             bbox=dict(boxstyle='round', fc='white', ec='red'))

    def update(frame):
        s_now = s_frames[frame]
        i = int(np.argmin(np.abs(s - s_now)))
        _place_patches(ego_patches, ax_top, x[i], y[i], yaw[i])
        cursor.set_xdata([s_now, s_now])
        speed_text.set_text(f'v = {v_frames_kph[frame]:.0f} km/h')
        return ego_patches + [cursor, speed_text]

    anim = FuncAnimation(fig, update, frames=len(s_frames), interval=interval, blit=False)
    plt.close(fig)
    return anim
