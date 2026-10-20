#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_4_setroutemany.py
# Description: Spawn vehicles periodically and follow the route
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  13, 2026: Seokhwan Jeong - Created.
#      Feb  21, 2026: Seokhwan Jeong - Spawn periodically + set_route()
#      Jul  03, 2026: Jisang Yun - TODO 주석/변수명 규칙 통일 (노트북 스타일)
#======================================================================#

import carla
import random

MAX_VEHICLES = 50

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

# ============================================================
# 차량 Blueprint와 Spawn point 불러오기
# ============================================================
vehicle_blueprints = world.get_blueprint_library().filter("vehicle.*")
spawn_points = world.get_map().get_spawn_points()

# ============================================================
# Spawn 시스템
# ============================================================
spawn_point = spawn_points[32]

##### TODO 1 #####
# 두 번째 경로용 spawn point 를 하나 더 고르기
# (예: spawn_points[92])
#################

spawn_point2 = ...

#################

# Spawn 주기 설정
spawn_interval_sec = 5.0
spawn_interval_ticks = max(1, int(round(spawn_interval_sec / 0.05)))
spawn_tick_counter = 0

spawned_vehicles = []

try:
    while True:
        world.tick()

        # 월드 내 차량 수 제한
        if len(spawned_vehicles) >= MAX_VEHICLES:
            continue

        spawn_tick_counter += 1
        if spawn_tick_counter < spawn_interval_ticks:
            continue
        spawn_tick_counter = 0

        # 차량 Spawn
        blueprint = random.choice(vehicle_blueprints)

        ##### TODO 2 #####
        # 두 번째 spawn_point2 위치에 두 번째 차량 스폰하기
        # 사용함수: world.try_spawn_actor(blueprint, transform) / 기능: 가능하면 actor 생성, 불가하면 None 반환
        #################

        vehicle2 = ...

        #################

        # 두 번째 차량은 첫 번째 차량 스폰 성공 여부와 상관없이 바로 설정 (아래 continue 에 걸려 방치되지 않도록)
        ##### TODO 3 #####
        # 두 번째 차량용 경로 명령 리스트 만들기 ("Left" / "Right" / "Straight")
        # 주의: 출발 위치가 다르면 첫 번째 차량의 명령이 안 맞을 수 있음
        # 주의: 그 교차로에 없는 방향 -> 'We couldn't find the RoadOption ...' 경고 + 명령과 다른 경로로 주행
        #################

        route_cmds2 = [..., ..., ..., ..., ...]

        #################

        ##### TODO 4 #####
        # 두 번째 차량(vehicle2)에 route_cmds2 경로 적용하기 (autopilot 은 이미 켜짐)
        # 사용함수: traffic_manager.set_route(vehicle2, route_cmds2) / 기능: 교차로 방향 지시대로 차량 경로 설정
        #################

        if vehicle2:
            vehicle2.set_autopilot(True, tm_port)
            spawned_vehicles.append(vehicle2.id)
            ...

        #################

        vehicle = world.try_spawn_actor(blueprint, spawn_point)

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

        traffic_manager.set_route(vehicle, route_cmds)

        spawned_vehicles.append(vehicle.id)

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
