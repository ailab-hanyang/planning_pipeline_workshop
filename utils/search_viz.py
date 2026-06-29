"""
Visualization helpers for grid search (Dijkstra / A* / Hybrid A*).

- animate_motion_model : 8방향 이동 모델을 화살표로 하나씩 보여주는 애니메이션
- animate_search       : 탐색 노드가 비용 순으로 퍼지고 경로를 역추적하는 애니메이션
- animate_compare      : 두 알고리즘의 탐색 과정을 좌우로 동시 재생 (예: Dijkstra vs A*)

모든 그림 텍스트는 영어 (matplotlib 한글 깨짐 방지).

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def animate_motion_model(motion, labels):
    """8방향 이동 모델: 화살표를 하나씩 추가하며 직선(파랑)/대각선(주황) 구분."""
    colors = ['royalblue'] * 4 + ['darkorange'] * 4

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(-2.3, 2.3); ax.set_ylim(-2.3, 2.3); ax.set_aspect('equal')
    ax.set_xticks([-1, 0, 1]); ax.set_yticks([-1, 0, 1])
    ax.grid(True, alpha=0.4)
    ax.axhline(0, color='k', lw=0.8); ax.axvline(0, color='k', lw=0.8)
    ax.plot(0, 0, 'o', color='royalblue', ms=16, zorder=5)
    ax.text(0, 0, 'Current\nNode', ha='center', va='center',
            fontsize=8, color='white', fontweight='bold', zorder=6)
    ax.set_title('8-Direction Motion Model (blue=straight, orange=diagonal)', fontsize=11)

    drawn = []
    status = ax.text(0, -2.15, '', ha='center', fontsize=10, color='dimgray')

    def update(frame):
        dx, dy, cost = motion[frame]
        color = colors[frame]; label = labels[frame]
        kind = 'straight' if cost == 1.0 else 'diagonal'
        arr = ax.annotate('', xy=(dx * 1.5, dy * 1.5), xytext=(0, 0),
                          arrowprops=dict(arrowstyle='->', color=color, lw=2.5))
        txt = ax.text(dx * 1.9, dy * 1.9, f'{label}\n{cost:.3f}', ha='center', va='center',
                      fontsize=8, color=color, fontweight='bold')
        drawn.extend([arr, txt])
        status.set_text(f'[{frame+1}/{len(motion)}]  {label}  ({kind}, cost={cost:.3f})')
        return drawn + [status]

    ani = FuncAnimation(fig, update, frames=len(motion), interval=700, repeat=False, blit=False)
    plt.close(fig)
    return ani


def _setup_axes(ax, ox, oy, sx, sy, gx, gy, title):
    ax.plot(ox, oy, '.k', ms=2, alpha=0.5, zorder=1)
    ax.plot(sx, sy, 'og', ms=11, zorder=5, label='Start')
    ax.plot(gx, gy, 'xb', ms=11, mew=2.5, zorder=5, label='Goal')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.2); ax.legend(loc='upper left', fontsize=8)
    ax.set_title(title)


def animate_search(explore_order, path_x, path_y, ox, oy, sx, sy, gx, gy,
                   title='Search', n_batches=60, show_count=True):
    """
    탐색 과정 애니메이션.
    explore_order : [(x, y, cost), ...] 탐색 확정 순서
    path_x/path_y : 최종 경로
    show_count    : 타이틀에 탐색 노드 개수를 표시할지 (False면 개수 숨김)
    Phase 1: 노드가 비용(plasma) 순으로 퍼짐 / Phase 2: 경로 역추적 / Phase 3: 완료
    """
    n = len(explore_order)
    batch = max(1, n // n_batches)
    n_explore = math.ceil(n / batch)
    n_path = len(path_x)
    total = n_explore + n_path + 8
    max_cost = explore_order[-1][2]
    path_len = sum(math.hypot(path_x[i+1]-path_x[i], path_y[i+1]-path_y[i])
                   for i in range(len(path_x)-1))

    fig, ax = plt.subplots(figsize=(8, 8))
    _setup_axes(ax, ox, oy, sx, sy, gx, gy, f'{title}: exploring...')
    scat = ax.scatter([], [], c=[], cmap='plasma', vmin=0, vmax=max_cost, s=20, alpha=0.7, zorder=2)
    path_line, = ax.plot([], [], '-r', lw=2.5, zorder=4)
    title_obj = ax.title
    bx, by, bc = [], [], []

    def update(frame):
        if frame < n_explore:
            s, e = frame * batch, min(frame * batch + batch, n)
            for k in range(s, e):
                bx.append(explore_order[k][0]); by.append(explore_order[k][1]); bc.append(explore_order[k][2])
            scat.set_offsets(np.c_[bx, by]); scat.set_array(np.array(bc))
            title_obj.set_text(f'{title}: exploring  {min(e, n):,}/{n:,} nodes'
                               if show_count else f'{title}: exploring...')
            path_line.set_data([], [])
        elif frame < n_explore + n_path:
            k = frame - n_explore + 1
            path_line.set_data(path_x[:k], path_y[:k])
            title_obj.set_text(f'{title}: tracing path  {k}/{n_path}'
                               if show_count else f'{title}: tracing path...')
        else:
            path_line.set_data(path_x, path_y)
            title_obj.set_text(f'{title}: done!  {n:,} nodes  |  path {path_len:.1f} m'
                               if show_count else f'{title}: done!  |  path {path_len:.1f} m')
        return scat, path_line, title_obj

    ani = FuncAnimation(fig, update, frames=total, interval=80, repeat=False, blit=False)
    plt.close(fig)
    return ani


def animate_compare(order_a, path_ax, path_ay, order_b, path_bx, path_by,
                    ox, oy, sx, sy, gx, gy, label_a='Dijkstra', label_b='A*', n_batches=40):
    """두 알고리즘의 탐색을 좌우로 동시 재생하여 탐색 범위/속도를 비교."""
    na, nb = len(order_a), len(order_b)
    batch_a = max(1, na // n_batches)
    batch_b = max(1, nb // n_batches)
    n_explore = max(math.ceil(na / batch_a), math.ceil(nb / batch_b))
    # 경로 그리기 단계는 ~12프레임으로 서브샘플 (애니메이션 용량 절감)
    n_path = 12
    path_stride_a = max(1, len(path_ax) // n_path)
    path_stride_b = max(1, len(path_bx) // n_path)
    total = n_explore + n_path + 6
    vmax = max(order_a[-1][2], order_b[-1][2])

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    fig.set_dpi(78)   # jshtml 임베드 용량 절감
    _setup_axes(axes[0], ox, oy, sx, sy, gx, gy, label_a)
    _setup_axes(axes[1], ox, oy, sx, sy, gx, gy, label_b)
    scat_a = axes[0].scatter([], [], c=[], cmap='plasma', vmin=0, vmax=vmax, s=18, alpha=0.7, zorder=2)
    scat_b = axes[1].scatter([], [], c=[], cmap='plasma', vmin=0, vmax=vmax, s=18, alpha=0.7, zorder=2)
    line_a, = axes[0].plot([], [], '-r', lw=2.5, zorder=4)
    line_b, = axes[1].plot([], [], '-r', lw=2.5, zorder=4)
    bufs = {'a': ([], [], []), 'b': ([], [], [])}

    def fill(order, batch, n, frame, buf):
        s, e = frame * batch, min(frame * batch + batch, n)
        for k in range(s, e):
            buf[0].append(order[k][0]); buf[1].append(order[k][1]); buf[2].append(order[k][2])
        return min(e, n)

    def update(frame):
        if frame < n_explore:
            ea = fill(order_a, batch_a, na, frame, bufs['a'])
            eb = fill(order_b, batch_b, nb, frame, bufs['b'])
            scat_a.set_offsets(np.c_[bufs['a'][0], bufs['a'][1]]); scat_a.set_array(np.array(bufs['a'][2]))
            scat_b.set_offsets(np.c_[bufs['b'][0], bufs['b'][1]]); scat_b.set_array(np.array(bufs['b'][2]))
            axes[0].title.set_text(f'{label_a}: {ea:,}/{na:,} nodes')
            axes[1].title.set_text(f'{label_b}: {eb:,}/{nb:,} nodes')
        else:
            p = frame - n_explore + 1
            ka, kb = min(p*path_stride_a, len(path_ax)), min(p*path_stride_b, len(path_bx))
            line_a.set_data(path_ax[:ka], path_ay[:ka]); line_b.set_data(path_bx[:kb], path_by[:kb])
            axes[0].title.set_text(f'{label_a}: {na:,} nodes (done)')
            axes[1].title.set_text(f'{label_b}: {nb:,} nodes (done)')
        return scat_a, scat_b, line_a, line_b

    fig.suptitle(f'{label_a} vs {label_b} — explored area comparison', fontsize=13)
    ani = FuncAnimation(fig, update, frames=total, interval=80, repeat=False, blit=False)
    plt.close(fig)
    return ani
