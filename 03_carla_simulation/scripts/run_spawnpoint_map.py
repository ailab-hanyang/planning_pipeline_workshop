#!/usr/bin/env python3
"""맵의 spawn point 분포를 top-view 그림으로 저장 (서버 연결만, 비동기).

실행: python3 run_spawnpoint_map.py   ->  spawn_points.png 생성
"""
import carla
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    spawn_points = world.get_map().get_spawn_points()
    xs = [sp.location.x for sp in spawn_points]
    ys = [sp.location.y for sp in spawn_points]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(xs, ys, s=12, c='steelblue')
    for i, sp in enumerate(spawn_points):
        ax.annotate(str(i), (sp.location.x, sp.location.y), fontsize=5, alpha=0.6)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_title(f'{world.get_map().name} — {len(spawn_points)} spawn points')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    fig.savefig('spawn_points.png', dpi=90, bbox_inches='tight')
    print(f'spawn point {len(spawn_points)}개 -> spawn_points.png 저장 (인덱스 라벨 포함)')


if __name__ == '__main__':
    main()
