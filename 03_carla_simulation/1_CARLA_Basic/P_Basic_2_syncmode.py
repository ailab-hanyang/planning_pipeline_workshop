#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_2_syncmode.py
# Description: Connect to CARLA server, set synchronous mode
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  16, 2026: Seokhwan Jeong - Created.
#      Feb  22, 2026: Seokhwan Jeong - Made Empty Template for Student Exercise
#      Jul  03, 2026: Jisang Yun - TODO 주석/변수명 규칙 통일 (노트북 스타일)
#======================================================================#
import carla

# ============================================================
# Client를 CARLA 시뮬레이터에 연결
# ============================================================
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)

# ============================================================
# Server에서 정보 읽어오기
# ============================================================
world = client.get_world()
original_settings = world.get_settings()   # 종료 시 복구용 (sync 모드를 켠 채 끝나면 서버가 멈춘 것처럼 보임)
weather = world.get_weather()
blueprint_library = world.get_blueprint_library()

# ============================================================
# Sync Mode를 키고 fixed delta time 설정하기
# ============================================================
##### TODO 1 #####
# Sync 모드를 켜고 fixed delta time 설정하기
# 1) 현재 world 설정을 가져오기   사용함수: world.get_settings() / 기능: 현재 월드 설정 객체 반환
# 2) synchronous_mode 를 True 로 설정
# 3) fixed_delta_seconds 를 0.05 로 설정
# 4) 설정을 world 에 적용         사용함수: world.apply_settings(settings) / 기능: 변경한 설정을 월드에 반영
#################

settings = ...
settings.synchronous_mode = ...
settings.fixed_delta_seconds = ...
world.apply_settings(...)

#################

try:
    while True:
        ##### TODO 2 #####
        # world 를 tick 시켜 시뮬레이션을 한 스텝 진행하기 (트리거)
        # 사용함수: world.tick() / 기능: 서버에 1 스텝 진행을 요청하고 다음 프레임까지 대기
        #################

        ...

        #################

except KeyboardInterrupt:
    print("\n[Ctrl+C] Bringing back original settings...")

finally:
    world.apply_settings(original_settings)
