#!/usr/bin/env python3
"""다수 차량 spawn + autopilot (synchronous mode). TM 속성은 run_tm_attribute.py 참고.

실행: python3 run_spawn_many.py     종료: Ctrl+C
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

        spawn_points = world.get_map().get_spawn_points()
        bps = [b for b in world.get_blueprint_library().filter('vehicle.*')
               if b.has_attribute('number_of_wheels')
               and int(b.get_attribute('number_of_wheels')) == 4]
        n = 0
        for sp in spawn_points[:NUM_VEHICLES]:
            v = world.try_spawn_actor(random.choice(bps), sp)
            if v is not None:
                v.set_autopilot(True, tm.get_port())
                n += 1
        print(f'{n}대 spawn + autopilot — Ctrl+C 로 종료')
        while True:
            world.tick()
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
