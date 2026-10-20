"""
Heuristic field visualization for step02_astar notebook (정답 코드 아님, 그림 전용).

긴 시각화 코드를 노트북에서 분리. 노트북은 각 함수를 한 줄로 호출.
plot 라벨은 영어(matplotlib 한글 폰트 깨짐 방지), 한국어 설명은 노트북 print() 로 유지.

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import numpy as np
import matplotlib.pyplot as plt


def PlotHeuristicField(heuristic_zero, heuristic_euclidean,
                       ox, oy, sx, sy, gx, gy,
                       min_x, max_x, min_y, max_y, x_width, y_width, resolution):
    """heuristic 없음(Zero) vs 있음(Euclidean) 의 h(n) 필드 비교 (어두울수록 목표에 가까움)."""
    gxi = (gx - min_x) / resolution
    gyi = (gy - min_y) / resolution

    class _Pt:                       # 격자 인덱스를 heuristic 함수에 넣기 위한 임시 점
        def __init__(self, x, y): self.x, self.y = x, y
    goal_pt = _Pt(gxi, gyi)

    fields = {
        'No heuristic (Zero) = Dijkstra': heuristic_zero,
        'With heuristic (Euclidean) = A*': heuristic_euclidean,
    }

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    for ax, (name, hfunc) in zip(axes, fields.items()):
        H = np.array([[hfunc(_Pt(ix, iy), goal_pt) for ix in range(x_width)]
                      for iy in range(y_width)])
        im = ax.imshow(H, origin='lower', extent=[min_x, max_x, min_y, max_y],
                       cmap='viridis_r', alpha=0.9)   # 값이 작을수록(목표에 가까울수록) 밝은 노랑
        ax.plot(ox, oy, '.k', ms=2, alpha=0.4)
        ax.plot(sx, sy, 'og', ms=10, label='Start'); ax.plot(gx, gy, 'xr', ms=10, mew=2.5, label='Goal')
        if np.ptp(H) > 0:
            plt.colorbar(im, ax=ax, label='h(n) [grid cells]  (bright = close to goal)', fraction=0.046)
        else:   # h = 0 으로 균일 -> 색 막대 대신 문구 표시
            ax.text(0.5, 0.06, 'h(n) = 0 everywhere', transform=ax.transAxes, ha='center', fontsize=11,
                    color='white', bbox=dict(boxstyle='round', fc='black', alpha=0.5))
        ax.set_aspect('equal'); ax.legend(loc='upper left', fontsize=8); ax.set_title(name)
    fig.suptitle('Heuristic Field h(n)  (A* follows this gradient toward goal)')
    plt.tight_layout(); plt.show()
