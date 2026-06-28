#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_6_spawnvehiclemany.py
# Description: Connect to CARLA server, spawn multiple vehicles with autopilot
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#======================================================================#
import carla
import random

NUM_VEHICLES = 50

def destroy_all_vehicles(world, client):
    vehicles = world.get_actors().filter("vehicle.*")
    ids = [v.id for v in vehicles]
    if ids:
        client.apply_batch([carla.command.DestroyActor(x) for x in ids])


client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
original_settings = world.get_settings()

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_synchronous_mode(True)
tm_port = traffic_manager.get_port()

# ============================================================
# 다수 차량 Spawn하기
# ============================================================
#-[TODO]-Get the spawn points of the map
#[key functions] get_map().get_spawn_points()
spawn_points = world.get_map().get_spawn_points()

try:
    #-[TODO]-Get the blueprint of the vehicles
    #[key functions] get_blueprint_library().filter("vehicle.*")
    vehicle_bps = world.get_blueprint_library().filter("vehicle.*")

    vehicles_spawned = 0

    for spawnpoint in spawn_points:
        if vehicles_spawned >= NUM_VEHICLES:
            break

        #-[TODO]-Get the blueprint of the vehicles randomly
        #[key functions] random.choice(blueprint)
        selected_bp = random.choice(list(vehicle_bps))

        #-[TODO]-Set the Color attribute of the vehicles
        #[key functions] set_attribute("color", "Choose color")
        if selected_bp.has_attribute("color"):
            selected_bp.set_attribute("color",
                f"{random.randint(0,255)},{random.randint(0,255)},{random.randint(0,255)}")

        #-[TODO]-Spawn the vehicle on the selected spawn point
        #[key functions] try_spawn_actor(blueprint, transform)
        vehicle = world.try_spawn_actor(selected_bp, spawnpoint)

        if vehicle is None:
            continue

        #-[TODO]-Set the autopilot of the vehicle
        #[key functions] set_autopilot(bool)
        vehicle.set_autopilot(True, tm_port)

        vehicles_spawned += 1

    print(f"Spawned {vehicles_spawned} vehicles")

    while True:
        world.tick()

except KeyboardInterrupt:
    print("\n[Ctrl+C] Destroying All vehicles...")
    destroy_all_vehicles(world, client)

finally:
    try:
        world.apply_settings(original_settings)
    except Exception:
        pass
