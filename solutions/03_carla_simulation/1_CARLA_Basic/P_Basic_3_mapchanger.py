#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_3_mapchanger.py
# Description: Connect to CARLA server and change the map
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#======================================================================#
import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# ============================================================
# 맵으로 변경하기
# ============================================================
#-[TODO]- Load a new map using load_world() function
#[key functions] load_world('Map Name')
# 사용 가능한 맵: Town01~Town07, Town10 등 (이론 자료 참고)
client.load_world('Town01')

#-[TODO]- Load a new map and specific layer using load_world() function
#[key functions] load_world('Map Name_Opt', map_layers=carla.MapLayer.????)
# 예시: Buildings, Ground, Foliage, StreetLights, All 등
client.load_world('Town02_Opt', map_layers=carla.MapLayer.All)
