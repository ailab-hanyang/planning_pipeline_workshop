#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_1_connectserver.py
# Description: Connect to CARLA server and get world information
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  13, 2026: Seokhwan Jeong - Created.
#      Feb  22, 2026: Seokhwan Jeong - Made Empty Template for Student Exercise
#      Jul  03, 2026: Jisang Yun - TODO 주석/변수명 규칙 통일 (노트북 스타일)
#======================================================================#
# ─── CARLA 좌표계 (단위: m, 도) ──────────────────────────────
#   x = 차 앞(+),  y = 차 오른쪽(+),  z = 위(+)
#   yaw + = 위에서 볼 때 시계 방향 (우회전 쪽),  pitch - = 아래를 봄,  steer + = 우회전
#   ※ 교과서·ISO 8855 (y = 왼쪽, yaw + = 반시계) 와 좌우 반대
#   예) 위에서 내려다보기 -> pitch = -90 (P_Basic_7) / 차 뒤·위에서 따라보기 -> x = -8, z = 3 (P_Basic_8)
# ────────────────────────────────────────────────────────────
##### TODO 1 #####
# CARLA 라이브러리를 import 하기
# 사용함수: import carla / 기능: CARLA Python API 모듈 불러오기
#################

...

#################

# ============================================================
# Client를 CARLA 시뮬레이터에 연결
# ============================================================
##### TODO 2 #####
# IP 와 포트 번호로 Client 를 만들어 CARLA 서버에 연결하기
# 사용함수: carla.Client(ip, port) / 기능: 지정한 주소/포트의 CARLA 서버에 연결하는 Client 생성
#################

client = ...
client.set_timeout(5.0)

#################

# ============================================================
# Server에서 정보 읽어오기
# ============================================================
##### TODO 3 #####
# 서버에서 world / weather / blueprint_library / spectator 정보 가져오기
# 사용함수: client.get_world(), world.get_weather(), world.get_blueprint_library(), world.get_spectator()
#          기능: 각각 월드, 날씨, blueprint 목록, 관전 카메라(spectator) 반환
#################

world = ...
weather = ...
blueprint_library = ...
spectator = ...

#################

# ============================================================
# Spectator 위치 변경하기
# ============================================================
##### TODO 4 #####
# carla.Location 과 carla.Rotation 으로 spectator 의 위치/자세 값 만들기
# 사용함수: carla.Location(x=, y=, z=) / carla.Rotation(pitch=, yaw=, roll=) / 기능: 위치·자세(회전) 값 생성
# 주의: Rotation 위치 인자 순서 = (pitch, yaw, roll) -> 항상 이름=값 으로 작성
#################

location = carla.Location(x=..., y=..., z=...)
rotation = carla.Rotation(roll=..., pitch=..., yaw=...)

#################

##### TODO 5 #####
# spectator 의 transform 설정하기
# 사용함수: spectator.set_transform(carla.Transform(location, rotation)) / 기능: 관전 카메라를 지정한 위치/자세로 이동
#################

...

#################
