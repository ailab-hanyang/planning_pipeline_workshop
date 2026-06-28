#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_3_setpath.py
# Description: Spawn vehicle and force it to follow a private path
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  21, 2026: Seokhwan Jeong - Created.
#======================================================================#

import carla
import random

def destroy_all_vehicles(world, client):
    vehicles = world.get_actors().filter("vehicle.*")
    ids = [v.id for v in vehicles]
    if ids:
        client.apply_batch([carla.command.DestroyActor(x) for x in ids])


# ============================================================
# Client를 CARLA 시뮬레이터에 연결
# ============================================================
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
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

#traffic_manager.set_random_device_seed(0)
#random.seed(0)

# ============================================================
# 차량 Blueprint / Spawn points 준비
# ============================================================
vehicle_bps = world.get_blueprint_library().filter("vehicle.*")
spawn_points = world.get_map().get_spawn_points()

spawn_point = spawn_points[32]
selected_bp = random.choice(vehicle_bps)

vehicle = None

try:
    # ============================================================
    # 차량 Spawn
    # ============================================================
    vehicle = world.try_spawn_actor(selected_bp, spawn_point)
    if not vehicle:
        raise RuntimeError(f"Fail to spawn vehicle.")

    # ============================================================
    # Autopilot ON
    # ============================================================
    vehicle.set_autopilot(True, tm_port)
    traffic_manager.update_vehicle_lights(vehicle, True)
    traffic_manager.auto_lane_change(vehicle, False)
    traffic_manager.ignore_lights_percentage(vehicle, 100)

    # ============================================================
    # Make path
    # ============================================================
    #-[TODO]- Make a list of spawn points index
    path_lists = [???, ???, ???, ???, ???]
    #Convert the spawn point index list to a list of carla.Location
    path_locations = [spawn_points[i].location for i in path_lists]
    #-[TODO]- Set the path to follow using set_path() function of Traffic Manager
    #[key features] set_path(vehicle, path_locations)
    traffic_manager.

    while True:
        world.tick()

except KeyboardInterrupt:
    print("\n[Exit] Ctrl+C detected. Destroying ALL vehicles...")

finally:
    try:
        destroy_all_vehicles(world, client)
    except Exception as e:
        print(f"[Destroy] Failed: {e}")

    try:
        traffic_manager.set_synchronous_mode(False)
    except Exception:
        pass

    try:
        world.apply_settings(original_settings)
    except Exception:
        pass