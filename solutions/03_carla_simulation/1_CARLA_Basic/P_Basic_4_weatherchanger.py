#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_4_weatherchanger.py
# Description: Connect to CARLA server and change the weather
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#======================================================================#
import carla

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()

# ============================================================
# 날씨 설정하기
# ============================================================
weather = world.get_weather()

#-[TODO]- Set weather parameters (각 값의 범위: 0~100, sun_altitude_angle: -90~90, sun_azimuth_angle: 0~360)
#[key functions] cloudiness, precipitation, precipitation_deposits, wetness, sun_altitude_angle, sun_azimuth_angle...
weather.cloudiness = 80.0
weather.precipitation = 100.0
weather.precipitation_deposits = 80.0
weather.wetness = 90.0
weather.sun_altitude_angle = 60.0
weather.sun_azimuth_angle = 120.0

#-[TODO]- Set weather configuration using set_weather() function
#[key functions] .set_weather(weather)
world.set_weather(weather)
