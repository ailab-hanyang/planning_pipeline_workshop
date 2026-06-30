#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_2_spawnpointchecker.py
# Description: Get the spawn points, draw them in the map
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  17, 2026: Seokhwan Jeong - Created.
#      Feb  18, 2026: Seokhwan Jeong - Added DebugString
#======================================================================#
import carla

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
# Spawn point 확인하기
# ============================================================
spawn_points = world.get_map().get_spawn_points()
#Draw the spawn points in the map
for i, spawn_point in enumerate(spawn_points):
    #-[TODO]-use debug function draw_sring() 
    #[key features] debug.draw_string(location, str(i), life_time=second)
    world.debug.draw_string(spawn_point.location, str(i), life_time=100.0)

try:
    while True:
        world.tick()
except KeyboardInterrupt:
    print("\n[Ctrl+C] Bringing back original settings...")
finally:
    try:
        world.apply_settings(original_settings)
    except Exception:
        pass