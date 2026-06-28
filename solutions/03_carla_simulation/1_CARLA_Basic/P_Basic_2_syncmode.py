#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_2_syncmode.py
# Description: Connect to CARLA server, set synchronous mode
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#======================================================================#
import carla

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
weather = world.get_weather()
blueprint_library = world.get_blueprint_library()

# ============================================================
# Sync Mode를 키고 fixed delta time 설정하기
# ============================================================
#-[TODO]- Get current world settings using get_settings() function
#[key functions] get_settings()
settings = world.get_settings()
#-[TODO]- set synchronous_mode to True
settings.synchronous_mode = True
#-[TODO]- set fixed_delta_seconds to 0.05
settings.fixed_delta_seconds = 0.05
#-[TODO]- apply the settings to the world using apply_settings() function
#[key functions] apply_settings(settings)
world.apply_settings(settings)

while True:
    #-[TODO]- Tick the world to update the simulation (trigger)
    #[key functions] tick()
    world.tick()
