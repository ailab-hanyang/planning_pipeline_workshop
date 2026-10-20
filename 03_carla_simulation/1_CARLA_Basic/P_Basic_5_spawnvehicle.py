#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_5_spawnvehicle.py
# Description: Practice code for spawning a vehicle
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  17, 2026: Seokhwan Jeong - Created.
#      Feb  22, 2026: Seokhwan Jeong - Made Empty Template for Student Exercise
#      Jul  03, 2026: Jisang Yun - TODO 주석/변수명 규칙 통일 (노트북 스타일)
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
tm_port = traffic_manager.get_port()   # tm_port: Traffic Manager 포트 번호

# ============================================================
# Ego 차량 Spawn하기
# ============================================================
##### TODO 1 #####
# Ego 차량의 blueprint 가져오기 (차량 ID 목록: https://carla.readthedocs.io/en/0.9.16/bp_library/#vehicle)
# 사용함수: world.get_blueprint_library().find(blueprint_id) / 기능: blueprint_id 에 해당하는 actor blueprint 반환
#################

ego_blueprint = world.get_blueprint_library().find(...)

#################

##### TODO 2 #####
# Ego 차량의 속성(attribute) 설정하기 (역할 이름 role_name, 색상 color)
# 사용함수: blueprint.set_attribute(name, value) / 기능: blueprint 의 속성 값을 지정
#################

ego_blueprint.set_attribute('role_name', ...)
ego_blueprint.set_attribute('color', ...)

#################

##### TODO 3 #####
# 맵의 스폰 지점(spawn point) 목록 가져오기
# 사용함수: world.get_map().get_spawn_points() / 기능: 맵에 정의된 스폰 지점(Transform) 리스트 반환
#################

spawn_points = ...

#################

try:
    ##### TODO 4 #####
    # spawn_points 리스트에서 스폰할 지점 하나 고르기
    # (예: spawn_points[0])
    #################

    ego_transform = ...

    #################

    ##### TODO 5 #####
    # Ego 차량 스폰하기
    # 사용함수: world.spawn_actor(blueprint, transform) / 기능: blueprint 를 transform 위치에 actor 로 생성해 반환
    #################

    ego_vehicle = ...

    #################

    ##### TODO 6 #####
    # Ego 차량을 autopilot 으로 설정하기
    # 사용함수: vehicle.set_autopilot(enabled) / 기능: 차량 autopilot on/off
    #################

    ...

    #################

    while True:
        world.tick()

except KeyboardInterrupt:
    print("\n[Ctrl+C] Destroying vehicle...")
    ##### TODO 7 #####
    # Ego 차량 제거(destroy)하기
    # 사용함수: actor.destroy() / 기능: 시뮬레이터에서 해당 actor 제거
    #################

    ...

    #################

finally:
    try:
        world.apply_settings(original_settings)
    except Exception:
        pass
