#!/usr/bin/env python3
"""키보드로 ego 수동 제어 (WASD). 비동기 모드 + pygame 입력.

W=가속 S=후진 A/D=조향 SPACE=브레이크 ESC=종료
실행: python3 run_manual_control.py
"""
import carla
import pygame


def main():
    pygame.init()
    pygame.display.set_mode((400, 120))
    pygame.display.set_caption('CARLA manual control (W/A/S/D, SPACE, ESC)')

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    bp.set_attribute('role_name', 'ego')
    ego = world.try_spawn_actor(bp, world.get_map().get_spawn_points()[0])
    if ego is None:
        print('spawn 실패'); return

    clock = pygame.time.Clock()
    try:
        running = True
        while running:
            clock.tick(30)
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
            keys = pygame.key.get_pressed()
            if keys[pygame.K_ESCAPE]:
                running = False
            ctrl = carla.VehicleControl()
            ctrl.throttle = 0.6 if keys[pygame.K_w] else 0.0
            ctrl.reverse = keys[pygame.K_s]
            if keys[pygame.K_s]:
                ctrl.throttle = 0.6
            ctrl.steer = (-0.5 if keys[pygame.K_a] else 0.0) + (0.5 if keys[pygame.K_d] else 0.0)
            ctrl.brake = 1.0 if keys[pygame.K_SPACE] else 0.0
            ego.apply_control(ctrl)

            # spectator를 ego 뒤로
            tf = ego.get_transform()
            import math
            yaw = math.radians(tf.rotation.yaw)
            cam = carla.Transform(
                tf.location + carla.Location(x=-7 * math.cos(yaw), y=-7 * math.sin(yaw), z=3),
                carla.Rotation(pitch=-12, yaw=tf.rotation.yaw))
            world.get_spectator().set_transform(cam)
    finally:
        try:
            ego.destroy()
        except Exception:
            pass
        pygame.quit()
        print('종료')


if __name__ == '__main__':
    main()
