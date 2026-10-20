#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_3_mapchanger.py
# Description: Connect to CARLA server and change the map
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
client.set_timeout(10.0)

# ============================================================
# 맵으로 변경하기
# ============================================================
##### TODO 1 #####
# load_world() 로 새 맵 로드하기 (사용 가능한 맵: Town01~Town05, Town10HD 와 각각의 _Opt 버전, 이론 자료 참고)
# 사용함수: client.load_world(map_name) / 기능: 지정한 맵을 새로 로드
#################

client.load_world(...)

#################

##### TODO 2 #####
# load_world() 로 특정 레이어만 포함해 맵 로드하기 (예시 레이어: Buildings, Ground, Foliage, StreetLights, All)
# 사용함수: client.load_world(map_name, map_layers=carla.MapLayer.XXX) / 기능: 지정 맵을 특정 레이어와 함께 로드
# 주의: 레이어 지정은 이름이 _Opt 인 맵만 적용 (Town01 등 -> 레이어 무시, 전부 로드)
# 확인: Buildings 만 -> 건물은 그대로, 나무·주차 차량 없음 (All 과 비교)
#################

client.load_world(..., map_layers=...)

#################

# 확인: 건물만 남은 맵 (나무·주차 차량 없음) -> Enter 로 전체 레이어 복구 (다음 실습용)
input('[INFO] 레이어 확인 후 Enter -> 전체 레이어로 복구')
client.load_world('Town02_Opt')   # 레이어 지정 없음 = 전체 레이어
