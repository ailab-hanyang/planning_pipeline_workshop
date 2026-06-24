#!/usr/bin/env python3
"""Step 03: ego 차량 spawn + autopilot 주행 + spectator 추종 (synchronous mode).

실행: python3 run_spawn_ego.py    (CARLA 0.9.16 서버가 켜져 있어야 함)
종료: Ctrl+C
"""
import carla
import math


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    original_settings = world.get_settings()
    ego = None
    tm = None
    try:
        # 동기 모드 + Traffic Manager
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)

        # ego 블루프린트 + spawn
        bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        bp.set_attribute('role_name', 'ego')
        if bp.has_attribute('color'):
            bp.set_attribute('color', '200,0,0')
        spawn_point = world.get_map().get_spawn_points()[0]
        ego = world.try_spawn_actor(bp, spawn_point)
        if ego is None:
            raise RuntimeError('spawn 실패 — 다른 spawn point를 시도하세요')
        ego.set_autopilot(True, tm.get_port())
        print('ego 주행 시작 — 창에서 확인하세요. Ctrl+C 로 종료')

        # 동기 루프: 매 틱 진행 + spectator를 ego 뒤·위로
        while True:
            world.tick()
            tf = ego.get_transform()
            yaw = math.radians(tf.rotation.yaw)
            cam_loc = carla.Location(x=tf.location.x - 8 * math.cos(yaw),
                                     y=tf.location.y - 8 * math.sin(yaw),
                                     z=tf.location.z + 4)
            cam_rot = carla.Rotation(pitch=-15, yaw=tf.rotation.yaw)
            world.get_spectator().set_transform(carla.Transform(cam_loc, cam_rot))

    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        if ego is not None:
            try:
                ego.destroy()
            except Exception:
                pass
        if tm is not None:
            try:
                tm.set_synchronous_mode(False)
            except Exception:
                pass
        world.apply_settings(original_settings)
        print('정리 완료 (액터 제거 + 설정 복구)')


if __name__ == '__main__':
    main()
