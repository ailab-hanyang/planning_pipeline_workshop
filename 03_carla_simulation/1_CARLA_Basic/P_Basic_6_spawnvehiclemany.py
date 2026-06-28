#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_6_spawnvehiclemany.py
# Description: Connect to CARLA server, set synchronous mode, spawn ego vehicle and set autopilot
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  17, 2026: Seokhwan Jeong - Created.
#      Feb  18, 2026: Seokhwan Jeong - Added random selection
#      Feb  22, 2026: Seokhwan Jeong - Made Empty Template for Student Exercise
#======================================================================#
import carla
import random

NUM_VEHICLES = 50

def destroy_all_vehicles(world, client):
    vehicles = world.get_actors().filter("vehicle.*")
    ids = [v.id for v in vehicles]
    if ids:
        client.apply_batch([carla.command.DestroyActor(x) for x in ids])


# ============================================================
# Client를 CARLA 시뮬레이터에 연결
# ============================================================
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()

original_settings = world.get_settings()

# ============================================================
# Synchronous 모드 설정
# ============================================================
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# ============================================================
# Traffic Manager를 연결
# ============================================================
traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_synchronous_mode(True)
tm_port = traffic_manager.get_port()


# ============================================================
# Ego 차량 Spawn하기
# ============================================================
#-[TODO]-Get the spawn points of the map
#[key functions] get_map().get_spawn_points()
spawn_points = world.

try:
    #-[TODO]-Get the blueprint of the vehicles
    #[key functions] get_blueprint_library().filter("vehicle.*")
    vehicle_bps = world.

    # Counter for spawned vehicles
    vehicles_spawned = 0

    # For Loop to spawn multiple vehicles
    for spawnpoint in spawn_points:
        if vehicles_spawned >= NUM_VEHICLES:
            break
        
        #-[TODO]-Get the blueprint of the vehicles randomly
        #[key functions] random.choice(blueprint)
        selected_bp = 
        
        #-[TODO]-Set the Color attribute of the vehicles
        #[key functions] set_attribute("color", "Choose color")
        if selected_bp.has_attribute("color"):
            selected_bp.

        #-[TODO]-Spawn the vehicle on the selected spawn point
        #[key functions] try_spawn_actor(blueprint, transform)
        vehicle = 
        
        #-[TODO]-Set the autopilot of the vehicle
        #[key functions] set_autopilot(bool)
        vehicle.

        # Add Counter
        vehicles_spawned += 1

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