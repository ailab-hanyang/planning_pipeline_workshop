#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_1_attribute.py
# Description: Add Traffic Manager attribute to the spawned vehicles
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  17, 2026: Seokhwan Jeong - Created.
#      Feb  18, 2026: Seokhwan Jeong - Added random selection
#      Feb  21, 2026: Seokhwan Jeong - Added Traffic Manager attribute
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
#Get the spawn points of the map
spawn_points = world.get_map().get_spawn_points()

try:
    vehicle_bps = world.get_blueprint_library().filter("vehicle.*")
    vehicle_bps = [bp for bp in vehicle_bps
             if bp.has_attribute("number_of_wheels") and int(bp.get_attribute("number_of_wheels")) == 4]

    vehicles_spawned = 0
    for spawnpoint in spawn_points:
        if vehicles_spawned >= NUM_VEHICLES:
            break

        selected_bp = random.choice(vehicle_bps)
        if selected_bp.has_attribute("color"):
            selected_bp.set_attribute("color", random.choice(selected_bp.get_attribute("color").recommended_values))

        vehicle = world.try_spawn_actor(selected_bp, spawnpoint)
        if vehicle is None:
            continue
        vehicle.set_autopilot(True)
        #-[TODO]-Light up the spawned vehicles using update_vehicle_lights() function of Traffic Manager
        #[key features] update_vehicle_lights(vehicle, bool)
        traffic_manager.update_vehicle_lights(vehicle, True)

        vehicles_spawned += 1

    #-[TODO]-Set the lane offset attribute using global_lane_offset() function of Traffic Manager
    #[key features] global_lane_offset(offset meter)
    traffic_manager.global_lane_offset(0.5)

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