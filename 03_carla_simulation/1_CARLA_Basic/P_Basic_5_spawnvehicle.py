#!/usr/bin/env python3
#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_5_spawnvehicle.py
# Description: Practice code for spawning a vehicle
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  17, 2026: Seokhwan Jeong - Created.
#      Feb  22, 2026: Seokhwan Jeong - Made Empty Template for Student Exercise
#======================================================================#
import carla

# ============================================================
# Client를 CARLA 시뮬레이터에 연결
# ============================================================
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()

original_settings = world.get_settings()

# ============================================================
# Synchronous 모드 설정
# ============================================================
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# ============================================================
# Traffic Manager를 연결
# ============================================================
traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_synchronous_mode(True)
tm_port = traffic_manager.get_port()

# ============================================================
# Ego 차량 Spawn하기
# ============================================================
#-[TODO]- Get the blueprint of the ego vehicle
ego_bp = world.get_blueprint_library().find('CAR ID')
#-[TODO]- Set the attribute of the ego vehicle
ego_bp.set_attribute('role_name','???')
ego_bp.set_attribute('color',"???")

#-[TODO]- Get the spawn points of the map
#[key functions] get_map().get_spawn_points()
spawn_points = world.

try:
    #-[TODO]- Select the Spawnpoint from the spawn_point[] list
    ego_transform = 
    #-[TODO]- Spawn the ego vehicle
    #[key functions] spawn_actor(blueprint, transform)
    ego_vehicle = 
    #-[TODO]- Set the ego vehicle to autopilot
    #[key functions] set_autopilot(bool)
    ego_vehicle.

    while True:
        world.tick()

except KeyboardInterrupt:
    print("\n[Ctrl+C] Destroying vehicle...")
    #-[TODO]-Destroy ego vehicle
    #[key functions] destroy()
    ego_vehicle.


finally:
    try:
        world.apply_settings(original_settings)
    except Exception:
        pass