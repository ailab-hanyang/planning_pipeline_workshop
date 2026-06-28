#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_7_manualcontrol.py
# Description: Control vehicle with manual input
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  20, 2026: Seokhwan Jeong - Created.
#      Feb  21, 2026: Seokhwan Jeong - Added Manual Input
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
#Get the blueprint of the ego vehicle
ego_bp = world.get_blueprint_library().find('vehicle.nissan.patrol_2021')
#Set the attribute of the ego vehicle
ego_bp.set_attribute('role_name','ego')
ego_bp.set_attribute('color',"255,0,0")

#Get the spawn points of the map
spawn_points = world.get_map().get_spawn_points()

try:
    #Select the Spawnpoint and spawn the ego vehicle
    ego_transform = spawn_points[0]
    ego_vehicle = world.spawn_actor(ego_bp, ego_transform)

    # ============================================================
    # Spectator Transform 설정
    # ============================================================
    spectator = world.get_spectator()
    #-[TODO]-Set the spectator Location to be 50meters above the ego vehicle (x, y same as ego, z + 50)
    top_location = carla.Location(
        x=ego_transform.???,
        y=ego_transform.???,
        z=ego_transform.???
    )
    #-[TODO]-Set the spectator Rotation to be looking down (roll=0, pitch=-90, yaw=0)
    top_rotation = carla.Rotation(
        roll=???,
        pitch=???,
        yaw=???
    )
    #-[TODO]-Create a Transform for the spectator using the top_location and top_rotation
    #[key functions] set_transform(carla.Transform(location, rotation))
    spectator.

    # ============================================================
    # Manual Control 입력받아서 차량 제어하기
    # ============================================================
    ego_vehicle.set_autopilot(False)

    #-[TODO]-Create a VehicleControl message with throttle, steer, brake, and reverse values
    #-steering (+ : right, - : left)
    control_input = carla.VehicleControl(
        throttle=???,
        steer=???,
        brake=???,
        reverse=bool(???)
    )

    print("[INFO] Manual control start")

    while True:
        #-[TODO]-use apply_control() to apply the control to the ego vehicle
        #[key functions] apply_control(control_input)
        ego_vehicle.
        world.tick()

except KeyboardInterrupt:
    print("\n[Ctrl+C] Destroying vehicle...")

finally:
    try:
        if ego_vehicle is not None:
            ego_vehicle.destroy()
    except Exception:
        pass

    try:
        traffic_manager.set_synchronous_mode(False)
    except Exception:
        pass

    try:
        world.apply_settings(original_settings)
    except Exception:
        pass

    print("[DONE] Vehicle destroyed + settings restored.")