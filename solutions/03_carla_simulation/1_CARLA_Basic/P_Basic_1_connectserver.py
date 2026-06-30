#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_1_connectserver.py
# Description: Connect to CARLA server and get world information
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#======================================================================#
#-[TODO]- Import CARLA Library
import carla

# ============================================================
# Client를 CARLA 시뮬레이터에 연결
# ============================================================
#-[TODO]- Create Client and connect to CARLA server using IP and Port Number
#[key functions] carla.Client(IP, port)
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)

# ============================================================
# Server에서 정보 읽어오기
# ============================================================
#-[TODO]- Get information from the server using get_..() functions
#[key functions] get_world(), get_weather(), get_blueprint_library(), get_spectator()
world = client.get_world()
weather = world.get_weather()
blueprint_library = world.get_blueprint_library()
spectator = world.get_spectator()

# ============================================================
# Spectator 위치 변경하기
# ============================================================
#-[TODO]- Use carla.Location & .Rotation to set the spectator's position and orientation
transform = carla.Location(x=0.0, y=0.0, z=3.0)
rotation = carla.Rotation(roll=0.0, pitch=0.0, yaw=0.0)
#-[TODO]- Set the spectator's transform
#[key functions] set_transform(carla.Transform(Location, Rotation))
spectator.set_transform(carla.Transform(transform, rotation))
