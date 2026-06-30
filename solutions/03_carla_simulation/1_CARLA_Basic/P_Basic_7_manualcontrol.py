#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_7_manualcontrol.py
# Description: Control vehicle with manual input
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#======================================================================#
import carla

# ============================================================
# -[TODO]- Spectator가 Ego 차량을 따라다니도록 시점 설정하기
# ============================================================
def spectator_follow_tf(ego_transform):
    #-[TODO]-Add offset to the ego vehicle location to set the spectator location (x: -8.0, y: 0.0, z: 3.0)
    offset = carla.Location(x=-8.0, y=0.0, z=3.0)
    #-[TODO]-Set Camera location on ego frame
    #[key functions] transform(offset)
    cam_location = ego_transform.transform(offset)
    #-[TODO]-Set the spectator rotation to look at the ego vehicle (roll: 0.0, pitch: -12.0, yaw: same as ego)
    cam_rotation = carla.Rotation(roll=0.0, pitch=-12.0, yaw=ego_transform.rotation.yaw)
    return carla.Transform(cam_location, cam_rotation)

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
        x=ego_transform.location.x,
        y=ego_transform.location.y,
        z=ego_transform.location.z + 50.0
    )
    #-[TODO]-Set the spectator Rotation to be looking down (roll=0, pitch=-90, yaw=0)
    top_rotation = carla.Rotation(
        roll=0.0,
        pitch=-90.0,
        yaw=0.0
    )
    #-[TODO]-Create a Transform for the spectator using the top_location and top_rotation
    #[key functions] set_transform(carla.Transform(location, rotation))
    spectator.set_transform(carla.Transform(top_location, top_rotation))

    # ============================================================
    # Manual Control 입력받아서 차량 제어하기
    # ============================================================
    ego_vehicle.set_autopilot(False)

    #-[TODO]-Create a VehicleControl message with throttle, steer, brake, and reverse values
    #-steering (+ : right, - : left)
    control_input = carla.VehicleControl(
        throttle=0.5,
        steer=0.5,
        brake=0.0,
        reverse=bool(False)
    )

    print("[INFO] Manual control start")

    while True:
        #-[TODO]-use apply_control() to apply the control to the ego vehicle
        #[key functions] apply_control(control_input)
        ego_vehicle.apply_control(control_input)

        # Spectator가 차량을 따라다니도록 매 tick마다 업데이트
        spectator.set_transform(spectator_follow_tf(ego_vehicle.get_transform()))

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
