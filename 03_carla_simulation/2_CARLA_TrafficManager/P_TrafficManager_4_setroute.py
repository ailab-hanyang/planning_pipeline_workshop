#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_4_setroute.py
# Description: Make route for a vehicle using Traffic Manager set_route()
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
# 차량 Blueprint / Spawn point 준비
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
    # Make route
    # ============================================================
    ##### TODO 1 #####
    # 경로의 교차로마다 "Left" / "Right" / "Straight" 명령을 리스트로 작성하기
    # 주의: 그 교차로에 없는 방향 -> 'We couldn't find the RoadOption ...' 경고 + 명령과 다른 경로로 주행
    #################

    route_cmds = [..., ..., ..., ..., ...]

    #################

    ##### TODO 2 #####
    # set_route() 로 route_cmds 를 차량에 적용하기
    # 사용함수: traffic_manager.set_route(vehicle, route_cmds) / 기능: 교차로에서의 방향 지시 리스트대로 차량 경로 설정
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
