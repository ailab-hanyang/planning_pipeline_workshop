#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_3_mapchanger.py
# Description: Connect to CARLA server and change the map
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
# 맵으로 변경하기
# ============================================================
#-[TODO]- Load a new map using load_world() function
#[key functions] load_world('Map Name')
client.
#-[TODO]- Load a new map and specific layer using load_world() function
#[key functions] load_world('Map Name', map_layers=carla.MapLayer.????)
#client.
