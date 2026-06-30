#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_1_connectserver.py
# Description: Connect to CARLA server and get world information
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  13, 2026: Seokhwan Jeong - Created.
#      Feb  22, 2026: Seokhwan Jeong - Made Empty Template for Student Exercise
#======================================================================#
#-[TODO]- Import CARLA Library

# ============================================================
# Client를 CARLA 시뮬레이터에 연결
# ============================================================
#-[TODO]- Create Client and connect to CARLA server using IP and Port Number
#[key functions] carla.Client(IP, port)

# ============================================================
# Server에서 정보 읽어오기
# ============================================================
#-[TODO]- Get information from the server using get_..() functions
#[key functions] get_world(), get_weather(), get_blueprint_library(), get_spectator()
world = client.
weather = world.
blueprint_library = world.
spectator = world.

# ============================================================
# Spectator 위치 변경하기
# ============================================================
#-[TODO]- Use carla.Location & .Rotation to set the spectator's position and orientation
transform = carla.Location(x=???, y=???, z=???)
rotation = carla.Rotation(roll=???, pitch=???, yaw=???)
#-[TODO]- Set the spectator's transform
#[key functions] set_transform(carla.Transform(Location, Rotation))
spectator.
