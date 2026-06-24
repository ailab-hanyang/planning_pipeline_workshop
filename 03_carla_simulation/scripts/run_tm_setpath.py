#!/usr/bin/env python3
"""한 차량에 위치 기반 경로(set_path) 강제 (synchronous mode).

실행: python3 run_tm_setpath.py     종료: Ctrl+C
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
        vehicle = world.try_spawn_actor(bp, spawn_points[32 % len(spawn_points)])
        if vehicle is None:
            raise RuntimeError('spawn 실패')

        vehicle.set_autopilot(True, tm.get_port())
        tm.update_vehicle_lights(vehicle, True)
        tm.auto_lane_change(vehicle, False)
        tm.ignore_lights_percentage(vehicle, 100)

        # spawn point 인덱스들을 경유 지점으로 전달
        path_idx = [50, 60, 70, 80]
        path_locations = [spawn_points[i % len(spawn_points)].location for i in path_idx]
        tm.set_path(vehicle, path_locations)
        print('set_path 경로 부여 — Ctrl+C 로 종료')

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
