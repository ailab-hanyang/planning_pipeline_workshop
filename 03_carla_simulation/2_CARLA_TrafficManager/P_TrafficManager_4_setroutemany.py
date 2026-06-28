#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_4_setroutemany.py
# Description: Spawn vehicles periodically and follow the route
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  13, 2026: Seokhwan Jeong - Created.
#      Feb  21, 2026: Seokhwan Jeong - Spawn periodically + set_route()
#======================================================================#

import carla
import random

MAX_VEHICLES = 50

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
# 차량 Blueprint와 Spawn point 불러오기
# ============================================================
vehicle_bps = world.get_blueprint_library().filter("vehicle.*")
spawn_points = world.get_map().get_spawn_points()

# ============================================================
# Spawn 시스템
# ============================================================
spawn_point = spawn_points[32]
#-[TODO]-make new spawn_point for second route
#spawn_point2 = ???

#Spawn interval setting
spawn_interval_sec = 5.0
spawn_interval_ticks = max(1, int(round(spawn_interval_sec / 0.05)))
spawn_tick_counter = 0

spawned_vehicles = []

try:
    while True:
        world.tick()

        #Limit the number of vehicles in the world
        if len(spawned_vehicles) >= MAX_VEHICLES:
            continue

        spawn_tick_counter += 1
        if spawn_tick_counter < spawn_interval_ticks:
            continue
        spawn_tick_counter = 0

        #Spawn Vehicle
        bp = random.choice(vehicle_bps)
        #-[TODO]-spawn second vehicle using new spawn_point
        vehicle = world.try_spawn_actor(bp, spawn_point)
        #vehicle =

        if not vehicle:
            print("[SPAWN] Blocked. Waiting for next interval")
            continue

        # ============================================================
        # Autopilot ON
        # ============================================================
        vehicle.set_autopilot(True, tm_port)
        traffic_manager.update_vehicle_lights(vehicle, True)

        traffic_manager.auto_lane_change(vehicle, False)
        traffic_manager.ignore_lights_percentage(vehicle, 100.0)

        # ============================================================
        # set_route 적용
        # ============================================================
        route_cmds = ["Straight", "Right", "Straight", "Right", "Left"]
        #-[TODO]-make new route_cmds list for second route
        #route_cmds2 = 

        #-[TODO]-set route for second vehicle using route_cmds2
        traffic_manager.set_route(vehicle, route_cmds)
        #traffic_manager.

        spawned_vehicles.append(vehicle.id)

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