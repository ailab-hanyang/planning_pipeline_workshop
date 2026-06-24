#!/usr/bin/env python3
"""한 차량에 방향 명령 기반 경로(set_route) 강제 — 교차로 선택 (synchronous mode).

route_cmds: 'Left' / 'Right' / 'Straight' / 'LaneLeft' / 'LaneRight' 등
실행: python3 run_tm_setroute.py     종료: Ctrl+C
"""
import carla
import random


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    original_settings = world.get_settings()
    tm = None
    vehicle = None
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)

        spawn_points = world.get_map().get_spawn_points()
        bp = random.choice(world.get_blueprint_library().filter('vehicle.*'))
        vehicle = world.try_spawn_actor(bp, spawn_points[0])
        if vehicle is None:
            raise RuntimeError('spawn 실패')

        vehicle.set_autopilot(True, tm.get_port())
        tm.auto_lane_change(vehicle, False)
        tm.ignore_lights_percentage(vehicle, 100)

        route_cmds = ['Straight', 'Left', 'Straight', 'Right', 'Straight']
        tm.set_route(vehicle, route_cmds)
        print('set_route 경로 부여:', route_cmds, '— Ctrl+C 로 종료')

        while True:
            world.tick()
    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        if vehicle is not None:
            try:
                vehicle.destroy()
            except Exception:
                pass
        if tm is not None:
            try:
                tm.set_synchronous_mode(False)
            except Exception:
                pass
        world.apply_settings(original_settings)
        print('정리 완료')


if __name__ == '__main__':
    main()
