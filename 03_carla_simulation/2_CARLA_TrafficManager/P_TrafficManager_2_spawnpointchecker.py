#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_2_spawnpointchecker.py
# Description: Get the spawn points, draw them in the map
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  17, 2026: Seokhwan Jeong - Created.
#      Feb  18, 2026: Seokhwan Jeong - Added DebugString
#      Jul  03, 2026: Jisang Yun - TODO 주석/변수명 규칙 통일 (노트북 스타일)
#======================================================================#
import carla

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
# Spawn point 확인하기
# ============================================================
spawn_points = world.get_map().get_spawn_points()

# 맵 위에 스폰 지점 번호를 그려서 확인
for i, spawn_point in enumerate(spawn_points):
    ##### TODO 1 #####
    # debug.draw_string() 으로 각 스폰 지점 위치에 번호(i) 표시하기
    # 사용함수: world.debug.draw_string(location, text, life_time=seconds) / 기능: 지정 위치에 문자열을 일정 시간 동안 표시
    #################

    ...

    #################

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
