#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_5_spawnvehicle.py
# Description: Practice code for spawning a vehicle
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#======================================================================#
import carla

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
original_settings = world.get_settings()

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_synchronous_mode(True)
tm_port = traffic_manager.get_port()

# ============================================================
# Ego 차량 Spawn하기
# ============================================================
#-[TODO]- Get the blueprint of the ego vehicle
# 차량 ID 목록: https://carla.readthedocs.io/en/0.9.15/bp_library/#vehicle
ego_bp = world.get_blueprint_library().find('vehicle.audi.etron')
#-[TODO]- Set the attribute of the ego vehicle
ego_bp.set_attribute('role_name', 'ego')
ego_bp.set_attribute('color', '100,100,150')

#-[TODO]- Get the spawn points of the map
#[key functions] get_map().get_spawn_points()
spawn_points = world.get_map().get_spawn_points()

try:
    #-[TODO]- Select the Spawnpoint from the spawn_point[] list
    ego_transform = spawn_points[0]
    #-[TODO]- Spawn the ego vehicle
    #[key functions] spawn_actor(blueprint, transform)
    ego_vehicle = world.spawn_actor(ego_bp, ego_transform)
    #-[TODO]- Set the ego vehicle to autopilot
    #[key functions] set_autopilot(bool)
    ego_vehicle.set_autopilot(True)

    while True:
        world.tick()

except KeyboardInterrupt:
    print("\n[Ctrl+C] Destroying vehicle...")
    #-[TODO]-Destroy ego vehicle
    #[key functions] destroy()
    ego_vehicle.destroy()

finally:
    try:
        world.apply_settings(original_settings)
    except Exception:
        pass
