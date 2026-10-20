#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_6_spawnvehiclemany.py
# Description: Connect to CARLA server, spawn multiple vehicles with autopilot
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  17, 2026: Seokhwan Jeong - Created.
#      Feb  18, 2026: Seokhwan Jeong - Added random selection
#      Feb  22, 2026: Seokhwan Jeong - Made Empty Template for Student Exercise
#      Jul  03, 2026: Jisang Yun - TODO 주석/변수명 규칙 통일 (노트북 스타일)
#======================================================================#
import carla
import random

NUM_VEHICLES = 50

def destroy_all_vehicles(world, client):
    vehicles = world.get_actors().filter("vehicle.*")
    vehicle_ids = [vehicle.id for vehicle in vehicles]
    if vehicle_ids:
        client.apply_batch([carla.command.DestroyActor(vehicle_id) for vehicle_id in vehicle_ids])


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
tm_port = traffic_manager.get_port()   # tm_port: Traffic Manager 포트 번호

# ============================================================
# 다수 차량 Spawn하기
# ============================================================
##### TODO 1 #####
# 맵의 스폰 지점(spawn point) 목록 가져오기
# 사용함수: world.get_map().get_spawn_points() / 기능: 맵에 정의된 스폰 지점(Transform) 리스트 반환
#################

spawn_points = ...

#################

try:
    ##### TODO 2 #####
    # 차량 blueprint 목록 가져오기
    # 사용함수: world.get_blueprint_library().filter("vehicle.*") / 기능: 'vehicle.*' 에 매칭되는 blueprint 목록 반환
    #################

    vehicle_blueprints = ...

    #################

    vehicles_spawned = 0   # 지금까지 스폰한 차량 수

    for spawn_point in spawn_points:
        if vehicles_spawned >= NUM_VEHICLES:
            break

        ##### TODO 3 #####
        # 차량 blueprint 를 무작위로 하나 고르기
        # 사용함수: random.choice(seq) / 기능: 시퀀스에서 무작위 원소 하나 선택
        #################

        selected_blueprint = ...

        #################

        ##### TODO 4 #####
        # 차량 색상(color) 속성 설정하기 (예: 무작위 R,G,B)
        # 사용함수: blueprint.set_attribute("color", "R,G,B") / 기능: blueprint 의 색상 속성 지정
        #################

        if selected_blueprint.has_attribute("color"):
            ...

        #################

        ##### TODO 5 #####
        # 선택한 스폰 지점에 차량 스폰하기 (실패 시 None 반환)
        # 사용함수: world.try_spawn_actor(blueprint, transform) / 기능: 가능하면 actor 생성, 불가하면 None 반환
        #################

        vehicle = ...

        #################

        if vehicle is None:
            continue

        ##### TODO 6 #####
        # 차량을 autopilot 으로 설정하기 (Traffic Manager 포트 지정)
        # 사용함수: vehicle.set_autopilot(enabled, tm_port) / 기능: 지정한 TM 포트로 차량 autopilot on/off
        #################

        ...

        #################

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
