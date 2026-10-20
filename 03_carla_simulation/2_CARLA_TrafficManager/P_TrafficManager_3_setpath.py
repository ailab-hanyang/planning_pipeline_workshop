#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_3_setpath.py
# Description: Spawn vehicle and force it to follow a private path
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  21, 2026: Seokhwan Jeong - Created.
#      Jul  03, 2026: Jisang Yun - TODO 주석/변수명 규칙 통일 (노트북 스타일)
#======================================================================#

import carla
import random

def destroy_all_vehicles(world, client):
    vehicles = world.get_actors().filter("vehicle.*")
    vehicle_ids = [vehicle.id for vehicle in vehicles]
    if vehicle_ids:
        client.apply_batch([carla.command.DestroyActor(vehicle_id) for vehicle_id in vehicle_ids])


# ============================================================
# Client를 CARLA 시뮬레이터에 연결
# ============================================================
client = carla.Client('localhost', 2000)
client.set_timeout(30.0)   # 맵 로드 시간 고려
client.load_world_if_different('Town10HD_Opt')   # spawn point 번호(32, 128 ...)는 Town10HD_Opt 기준 (다른 맵이면 번호 의미가 달라짐)
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
tm_port = traffic_manager.get_port()   # tm_port: Traffic Manager 포트 번호

#traffic_manager.set_random_device_seed(0)
#random.seed(0)

# ============================================================
# 차량 Blueprint / Spawn points 준비
# ============================================================
vehicle_blueprints = world.get_blueprint_library().filter("vehicle.*")
spawn_points = world.get_map().get_spawn_points()

spawn_point = spawn_points[32]
selected_blueprint = random.choice(vehicle_blueprints)

vehicle = None

try:
    # ============================================================
    # 차량 Spawn
    # ============================================================
    vehicle = world.try_spawn_actor(selected_blueprint, spawn_point)
    if not vehicle:
        raise RuntimeError("Fail to spawn vehicle.")

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
    ##### TODO 1 #####
    # 차량이 차례로 지나갈 spawn point 번호 리스트 만들기
    #   - 번호 범위: 0~154 (Town10HD_Opt) / 번호 위치는 P_TrafficManager_2_* 실행으로 확인
    #################

    path_indices = [..., ..., ..., ..., ...]

    #################

    # 인덱스 리스트를 carla.Location 리스트로 변환
    path_locations = [spawn_points[i].location for i in path_indices]

    ##### TODO 2 #####
    # set_path() 로 차량이 위 경로(path_locations)를 따르게 하기
    # 사용함수: traffic_manager.set_path(vehicle, path_locations) / 기능: 차량이 지정한 위치들을 순서대로 지나도록 경로 지정
    #################

    ...

    #################

    while True:
        world.tick()

except KeyboardInterrupt:
    print("\n[Exit] Ctrl+C detected. Destroying ALL vehicles...")

finally:
    try:
        destroy_all_vehicles(world, client)
    except Exception as error:
        print(f"[Destroy] Failed: {error}")

    try:
        traffic_manager.set_synchronous_mode(False)
    except Exception:
        pass

    try:
        world.apply_settings(original_settings)
    except Exception:
        pass
