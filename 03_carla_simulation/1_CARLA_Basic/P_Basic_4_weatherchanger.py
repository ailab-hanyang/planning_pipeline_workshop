#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_4_weatherchanger.py
# Description: Connect to CARLA server and change the weather
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  16, 2026: Seokhwan Jeong - Created.
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

# ============================================================
# 날씨 설정하기
# ============================================================
# 현재 날씨를 가져오기 (get_weather())
weather = world.get_weather()

##### TODO 1 #####
# 날씨 파라미터 설정하기 (범위: 대부분 0~100, sun_altitude_angle: -90~90, sun_azimuth_angle: 0~360)
# 설정 속성: cloudiness, precipitation, precipitation_deposits, wetness, sun_altitude_angle, sun_azimuth_angle
#################

weather.cloudiness = ...
weather.precipitation = ...
weather.precipitation_deposits = ...
weather.wetness = ...
weather.sun_altitude_angle = ...
weather.sun_azimuth_angle = ...

#################

##### TODO 2 #####
# set_weather() 로 위에서 설정한 날씨를 world 에 적용하기
# 사용함수: world.set_weather(weather) / 기능: 지정한 날씨 파라미터를 월드에 반영
#################

world.set_weather(...)

#################
