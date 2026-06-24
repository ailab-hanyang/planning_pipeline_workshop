#!/usr/bin/env python3
"""Step 04: 다수 차량 spawn + Traffic Manager 자동 주행 (synchronous mode).

실행: python3 run_tm_attribute.py
종료: Ctrl+C
"""
import carla
import random

NUM_VEHICLES = 30


def destroy_all_vehicles(world, client):
    ids = [v.id for v in world.get_actors().filter('vehicle.*')]
    if ids:
        client.apply_batch([carla.command.DestroyActor(x) for x in ids])


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    original_settings = world.get_settings()
    tm = None
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm_port = tm.get_port()

        spawn_points = world.get_map().get_spawn_points()
        vehicle_bps = [bp for bp in world.get_blueprint_library().filter('vehicle.*')
                       if bp.has_attribute('number_of_wheels')
                       and int(bp.get_attribute('number_of_wheels')) == 4]

        vehicles = []
        for sp in spawn_points[:NUM_VEHICLES]:
            bp = random.choice(vehicle_bps)
            if bp.has_attribute('color'):
                bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
            v = world.try_spawn_actor(bp, sp)
            if v is not None:
                v.set_autopilot(True, tm_port)
                tm.update_vehicle_lights(v, True)
                tm.auto_lane_change(v, True)
                tm.distance_to_leading_vehicle(v, 3.0)
                tm.vehicle_percentage_speed_difference(v, -10)
                vehicles.append(v)
        print(f'{len(vehicles)}대 주행 시작 — Ctrl+C 로 종료')

        while True:
            world.tick()
            if vehicles:
                tf = vehicles[0].get_transform()
                cam = carla.Transform(tf.location + carla.Location(z=40), carla.Rotation(pitch=-90))
                world.get_spectator().set_transform(cam)

    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        try:
            destroy_all_vehicles(world, client)
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
