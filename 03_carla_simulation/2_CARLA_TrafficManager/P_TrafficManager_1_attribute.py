#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_1_attribute.py
# Description: Add Traffic Manager attribute to the spawned vehicles
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  17, 2026: Seokhwan Jeong - Created.
#      Feb  18, 2026: Seokhwan Jeong - Added random selection
#      Feb  21, 2026: Seokhwan Jeong - Added Traffic Manager attribute
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
# 차량 Spawn하기
# ============================================================
# 맵의 스폰 지점 가져오기
spawn_points = world.get_map().get_spawn_points()

try:
    # 4륜 차량 blueprint 만 추리기
    vehicle_blueprints = world.get_blueprint_library().filter("vehicle.*")
    vehicle_blueprints = [blueprint for blueprint in vehicle_blueprints
             if blueprint.has_attribute("number_of_wheels") and int(blueprint.get_attribute("number_of_wheels")) == 4]

    vehicles_spawned = 0
    for spawn_point in spawn_points:
        if vehicles_spawned >= NUM_VEHICLES:
            break

        selected_blueprint = random.choice(vehicle_blueprints)
        if selected_blueprint.has_attribute("color"):
            selected_blueprint.set_attribute("color", random.choice(selected_blueprint.get_attribute("color").recommended_values))

        vehicle = world.try_spawn_actor(selected_blueprint, spawn_point)
        if vehicle is None:
            continue
        vehicle.set_autopilot(True)

        ##### TODO 1 #####
        # 스폰한 차량의 라이트 제어를 Traffic Manager 에 맡기기
        # 사용함수: traffic_manager.update_vehicle_lights(vehicle, True) / 기능: 상황에 맞춰 TM 이 라이트를 켜고 끔
        # 확인: 맑은 낮 -> 전조등 꺼짐(정상), 회전할 때 방향지시등 / 비·밤(P_Basic_4 날씨) -> 전조등 켜짐
        #################

        ...

        #################

        vehicles_spawned += 1

    ##### TODO 2 #####
    # global_lane_offset() 으로 전체 차량의 차선 오프셋(m) 설정하기
    # 사용함수: traffic_manager.global_lane_offset(offset_m) / 기능: 모든 차량을 차선 중앙에서 offset_m 만큼 이동 (+ = 오른쪽, - = 왼쪽)
    #################

    ...

    #################

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
