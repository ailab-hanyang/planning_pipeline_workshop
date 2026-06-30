#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_4_weatherchanger.py
# Description: Connect to CARLA server and change the weather
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  16, 2026: Seokhwan Jeong - Created.
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

# ============================================================
# 날씨 설정하기
# ============================================================
#Get current weather using get_weather() function
weather = world.get_weather()

#-[TODO]- Set weather parameters (ex: cloudiness, precipitation, precipitation_deposits, wetness, sun_altitude_angle, sun_azimuth_angle)
#[key functions] cloudiness, precipitation, precipitation_deposits, wetness, sun_altitude_angle, sun_azimuth_angle...
#강의 자료 내 각 값의 범위 참고
weather.

#-[TODO]- Set weather configuration using set_weather() function
#[key functions] .set_weather(weather)
world.