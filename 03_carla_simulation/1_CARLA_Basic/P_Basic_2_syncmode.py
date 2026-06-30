#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_2_syncmode.py
# Description: Connect to CARLA server, set synchronous mode
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  16, 2026: Seokhwan Jeong - Created.
#      Feb  22, 2026: Seokhwan Jeong - Made Empty Template for Student Exercise
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
weather = world.get_weather()
blueprint_library = world.get_blueprint_library()

# ============================================================
# Sync Mode를 키고 fixed delta time 설정하기
# ============================================================
#-[TODO]- Get current worrld settings using get_settings() function
#[key functions] get_settings()
settings = world.
#-[TODO]- set synchronous_mode to True
settings.synchronous_mode = 
#-[TODO]- set fixed_delta_seconds to 0.05
settings.fixed_delta_seconds = 
#-[TODO]- apply the settings to the world using apply_settings() function
#[key functions] apply_settings(settings)
world.

while True:
    #-[TODO]- Tick the world to update the simulation (trigger)
    #[key functions] tick()
    world.