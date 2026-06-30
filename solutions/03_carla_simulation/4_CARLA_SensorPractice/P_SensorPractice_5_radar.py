#!/usr/bin/env python3
#======================================================================#
# Module:      NGV_SensorPractice_5_radar.py
# Description: Sensor practice - Radar visualization (pygame)
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  21, 2026: Seokhwan Jeong - Created.
#======================================================================#
import random
import numpy as np

try:
    import pygame
    from pygame.locals import K_ESCAPE
except ImportError:
    raise RuntimeError("cannot import pygame, make sure pygame is installed")

import carla


# ============================================================
# 기본 설정값들
# ============================================================
WINDOW_RES = "1920x1080"
WINDOW_W, WINDOW_H = [int(x) for x in WINDOW_RES.split("x")]

NUM_NPC = 20

SPECTATOR_MODES = ["CHASE", "TOP", "FIRST_PERSON"]
SPEC_CFG = {
    "chase_back": 8.0,
    "chase_up": 3.0,
    "chase_pitch": -10.0,

    "top_z": 60.0,
    "top_pitch": -89.0,
    "top_yaw": 0.0,

    "fp_front": 1.8,
    "fp_up": 1.35,
    "fp_pitch": -2.0,

    "map_view_z": 220.0,
    "map_view_pitch": -89.0,
    "map_view_yaw": -90.0,
}

MANUAL_CFG = {
    "throttle": 0.70,
    "brake": 1.00,
    "steer_step": 0.03,
    "steer_return": 0.80,
}

RADAR_MODE = "VELOCITY"  # NORMAL / DEPTH / VELOCITY
RADAR_POINT_RADIUS = 2
RADAR_DEPTH_MIN = 0.0
RADAR_DEPTH_MAX = 80.0
RADAR_VEL_MIN = -20.0
RADAR_VEL_MAX = 20.0


# ============================================================
# Util
# ============================================================
def is_alive(actor):
    try:
        return actor is not None and actor.is_alive
    except Exception:
        return False

def compute_map_center_from_spawnpoints(spawn_points):
    if not spawn_points:
        return carla.Location(x=0.0, y=0.0, z=0.0)
    sx = sum(sp.location.x for sp in spawn_points)
    sy = sum(sp.location.y for sp in spawn_points)
    sz = sum(sp.location.z for sp in spawn_points)
    n = float(len(spawn_points))
    return carla.Location(x=sx/n, y=sy/n, z=sz/n)

def spectator_follow_tf(ego_tf, mode, spec_cfg):
    if mode == "CHASE":
        back = float(spec_cfg["chase_back"])
        up = float(spec_cfg["chase_up"])
        pitch = float(spec_cfg["chase_pitch"])
        offset = carla.Location(x=-back, y=0.0, z=up)
        cam_loc = ego_tf.transform(offset)
        cam_rot = carla.Rotation(roll=0.0, pitch=pitch, yaw=ego_tf.rotation.yaw)
        return carla.Transform(cam_loc, cam_rot)

    if mode == "TOP":
        top_z = float(spec_cfg["top_z"])
        top_pitch = float(spec_cfg["top_pitch"])
        top_yaw = float(spec_cfg["top_yaw"])
        loc = carla.Location(
            x=ego_tf.location.x,
            y=ego_tf.location.y,
            z=ego_tf.location.z + top_z
        )
        rot = carla.Rotation(roll=0.0, pitch=top_pitch, yaw=top_yaw)
        return carla.Transform(loc, rot)

    fp_front = float(spec_cfg["fp_front"])
    fp_up = float(spec_cfg["fp_up"])
    fp_pitch = float(spec_cfg["fp_pitch"])
    offset = carla.Location(x=fp_front, y=0.0, z=fp_up)
    cam_loc = ego_tf.transform(offset)
    cam_rot = carla.Rotation(roll=0.0, pitch=fp_pitch, yaw=ego_tf.rotation.yaw)
    return carla.Transform(cam_loc, cam_rot)

def map_view_tf_from_cfg(map_center, spec_cfg):
    z = float(spec_cfg["map_view_z"])
    pitch = float(spec_cfg["map_view_pitch"])
    yaw = float(spec_cfg["map_view_yaw"])
    return carla.Transform(
        carla.Location(x=map_center.x, y=map_center.y, z=map_center.z + z),
        carla.Rotation(roll=0.0, pitch=pitch, yaw=yaw)
    )

def build_vehicle_control(keys, steer_cache, reverse_toggle, manual_cfg):
    steer_step = float(manual_cfg["steer_step"])
    steer_return = float(manual_cfg["steer_return"])
    throttle_val = float(manual_cfg["throttle"])
    brake_val = float(manual_cfg["brake"])

    if keys[pygame.K_a]:
        steer_cache -= steer_step
    elif keys[pygame.K_d]:
        steer_cache += steer_step
    else:
        steer_cache *= steer_return

    steer_cache = max(-1.0, min(1.0, steer_cache))

    throttle = 0.0
    brake = 0.0
    if keys[pygame.K_w] and not keys[pygame.K_s]:
        throttle = throttle_val
    elif keys[pygame.K_s] and not keys[pygame.K_w]:
        brake = brake_val

    hand_brake = bool(keys[pygame.K_SPACE])

    ctrl = carla.VehicleControl(
        throttle=float(throttle),
        steer=float(steer_cache),
        brake=float(brake),
        hand_brake=hand_brake,
        reverse=bool(reverse_toggle)
    )
    return ctrl, steer_cache

def find_vehicle_bp(world, preferred_id: str):
    lib = world.get_blueprint_library()
    try:
        bp = lib.find(preferred_id)
        if bp is not None:
            return bp
    except Exception:
        pass
    bps = lib.filter("vehicle.*")
    if not bps:
        raise RuntimeError("No vehicle blueprints found.")
    return bps[0]


# ============================================================
# Ego 차량 소환하기
# ============================================================
def spawn_ego_vehicle(world, tm_port):
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found.")
    ego_transform = random.choice(spawn_points)

    #-[TODO]- Spawn ego vehicle (use find_vehicle_bp() and .spawn_actor())
    ego_bp = find_vehicle_bp(world, "vehicle.dodge.charger_2020")
    ego = world.spawn_actor(ego_bp, ego_transform)
    ego.set_autopilot(True, tm_port)

    return ego, ego_transform, spawn_points


def select_spawnpoints_with_min_gap(spawn_points, ego_transform, n, min_dist_from_ego, min_dist_between):
    def far_from_ego(sp):
        return sp.location.distance(ego_transform.location) > float(min_dist_from_ego)

    candidates = [sp for sp in spawn_points if far_from_ego(sp)]
    random.shuffle(candidates)

    selected = []
    for sp in candidates:
        if all(sp.location.distance(s.location) > float(min_dist_between) for s in selected):
            selected.append(sp)
            if len(selected) >= n:
                break
    return selected


def spawn_npc_vehicles(client, world, tm_port, ego_transform):
    if NUM_NPC <= 0:
        return []

    spawn_points = world.get_map().get_spawn_points()
    blueprints = world.get_blueprint_library().filter("vehicle.*")
    blueprints = [
        bp for bp in blueprints
        if bp.has_attribute("number_of_wheels") and int(bp.get_attribute("number_of_wheels")) == 4
    ]

    selected_sps = select_spawnpoints_with_min_gap(
        spawn_points=spawn_points,
        ego_transform=ego_transform,
        n=min(NUM_NPC, len(spawn_points)),
        min_dist_from_ego=8.0,
        min_dist_between=15.0
    )

    n = min(NUM_NPC, len(selected_sps))
    if n <= 0:
        return []

    batch = []
    for i in range(n):
        npc_bp = random.choice(blueprints)
        if npc_bp.has_attribute("color"):
            npc_bp.set_attribute("color", random.choice(npc_bp.get_attribute("color").recommended_values))
        if npc_bp.has_attribute("driver_id"):
            npc_bp.set_attribute("driver_id", random.choice(npc_bp.get_attribute("driver_id").recommended_values))
        npc_bp.set_attribute("role_name", "autopilot")

        batch.append(
            carla.command.SpawnActor(npc_bp, selected_sps[i]).then(
                carla.command.SetAutopilot(carla.command.FutureActor, True, tm_port)
            )
        )

    responses = client.apply_batch_sync(batch, True)
    npc_ids = []
    for r in responses:
        if not r.error:
            npc_ids.append(r.actor_id)
    return npc_ids


def apply_tm_safety_settings(world, traffic_manager, ego_vehicle, npc_ids):
    #traffic_manager.global_percentage_speed_difference(50.0)
    all_ids = [ego_vehicle.id] + list(npc_ids)
    for vid in all_ids:
        v = world.get_actor(vid)
        if v is None:
            continue
        traffic_manager.distance_to_leading_vehicle(v, 6.0)
        traffic_manager.auto_lane_change(v, False)

def radar_to_surface(radar_data, sensor_range_m):
    w, h = WINDOW_W, WINDOW_H
    img = np.zeros((w, h, 3), dtype=np.uint8)

    cx = int(w * 0.50)
    cy = int(h * 0.88)
    scale = (h * 0.95) / max(float(sensor_range_m), 1e-6)

    mode = str(RADAR_MODE).upper()
    r = int(RADAR_POINT_RADIUS)

    for det in radar_data:
        az = det.azimuth
        depth = det.depth
        vel = det.velocity

        x = depth * np.cos(az)
        y = depth * np.sin(az)

        px = int(cx + y * scale)
        py = int(cy - x * scale)
        if not (0 <= px < w and 0 <= py < h):
            continue

        if mode == "NORMAL":
            col = (255, 255, 255)

        elif mode == "DEPTH":
            dmin = float(RADAR_DEPTH_MIN)
            dmax = float(sensor_range_m)

            t = np.clip((depth - dmin) / max(dmax - dmin, 1e-6), 0.0, 1.0)
            g = int(t * 255)
            col = (g, g, g)

        else:  # VELOCITY
            max_abs = max(abs(RADAR_VEL_MIN), abs(RADAR_VEL_MAX), 1e-6)
            s = float(np.clip(vel / max_abs, -1.0, 1.0))
            a = int(abs(s) * 255)
            base = 255 - a
            if abs(s) < 0.05:
                col = (255, 255, 255)
            elif s < 0:
                col = (255, base, base)   # approaching: red
            else:
                col = (base, base, 255)   # receding: blue

        if r <= 0:
            img[px, py] = col
        else:
            x0 = max(px - r, 0)
            x1 = min(px + r + 1, w)
            y0 = max(py - r, 0)
            y1 = min(py + r + 1, h)
            img[x0:x1, y0:y1] = col

    return pygame.surfarray.make_surface(img)


# ============================================================
# Main
# ============================================================
def main():
    pygame.init()

    # ============================================================
    # Client를 CARLA 시뮬레이터에 연결
    # ============================================================
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    original_settings = world.get_settings()
    random.seed(0)

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
    traffic_manager.set_random_device_seed(0)
    tm_port = traffic_manager.get_port()

    # ============================================================
    # Variable 초기화
    # ============================================================
    actor_ids = []
    ego_vehicle = None

    spectator = None
    map_view_tf = None

    follow_enabled = True
    cam_mode_idx = 0
    manual_enabled = False
    reverse_toggle = False
    steer_cache = 0.0

    sensor = None
    surface = None
    radar_range_m = 80.0

    # pygame
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("NGV | RADAR | C:Cam | V:MapTop | Q:Manual | R:Reverse | ESC Quit")
    font = pygame.font.SysFont("Arial", 18)

    try:
        # ============================================================
        # 차량 Spawn (EGO + NPC)
        # ============================================================
        ego_vehicle, ego_tf, spawn_points = spawn_ego_vehicle(world, tm_port)
        actor_ids.append(ego_vehicle.id)

        npc_ids = spawn_npc_vehicles(client, world, tm_port, ego_tf)
        actor_ids.extend(npc_ids)

        # ============================================================
        # TM 안전 세팅 적용 (차간거리 설정 등)
        # ============================================================
        apply_tm_safety_settings(world, traffic_manager, ego_vehicle, npc_ids)

        # ============================================================
        # Spectator / Map view 준비
        # ============================================================
        spectator = world.get_spectator()
        map_center = compute_map_center_from_spawnpoints(spawn_points)
        map_view_tf = map_view_tf_from_cfg(map_center, SPEC_CFG)

        # ============================================================
        # [TODO] Spawn and attach Radar
        # ============================================================
        #-[TODO]- Get radar information from blueprint (use .get_blueprint_library().find())
        #[key functions] get_blueprint_library().find("sensor.????")
        # radar id: "sensor.other.radar"
        radar_blueprint = world.get_blueprint_library().find("sensor.other.radar")

        #-[TODO]- Set radar attribute (use .set_attribute())
        #[key functions] set_attribute("attribute", "value"))
        radar_blueprint.set_attribute('horizontal_fov', "30")
        radar_blueprint.set_attribute('vertical_fov', "10")
        radar_blueprint.set_attribute('range', "50.0")

        #-[TODO]- Set radar transform
        radar_transform = carla.Transform(carla.Location(x=2.0, y=0.0, z=1.0), carla.Rotation(roll=0.0, pitch=0.0, yaw=0.0))

        #-[TODO]- Spawn radar actor on ego_vehicle (use .spawn_actor())
        #[key functions] spawn_actor(blueprint, transform, attach_to=ego_vehicle)
        radar = world.spawn_actor(radar_blueprint, radar_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)

        #visualization용 range parameter 업데이트
        try:
            radar_range_m = float(radar.attributes.get("range", "80"))
        except Exception:
            radar_range_m = 80.0

        def radar_callback(radar_data):
            nonlocal surface
            surface = radar_to_surface(radar_data, radar_range_m)

        #-[TODO]- Listen to radar data stream (use .listen())
        #[key functions] use .listen(callback))
        radar.listen(radar_callback)

        sensor = radar

        # ============================================================
        # Main loop
        # ============================================================
        call_exit = False
        while True:
            world.tick()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    mods = pygame.key.get_mods()

                    if event.key == K_ESCAPE or (event.key == pygame.K_q and (mods & pygame.KMOD_CTRL)):
                        call_exit = True
                    elif event.key == pygame.K_c:
                        cam_mode_idx = (cam_mode_idx + 1) % len(SPECTATOR_MODES)
                    elif event.key == pygame.K_v:
                        follow_enabled = not follow_enabled
                        if not follow_enabled and spectator is not None and map_view_tf is not None:
                            spectator.set_transform(map_view_tf)
                    elif event.key == pygame.K_q:
                        manual_enabled = not manual_enabled
                        reverse_toggle = False
                        steer_cache = 0.0
                        if manual_enabled:
                            ego_vehicle.set_autopilot(False, tm_port)
                            print("[AUTOPILOT] OFF, MANUAL CONTROL ENABLED")
                        else:
                            ego_vehicle.set_autopilot(True, tm_port)
                            print("[AUTOPILOT] ON, MANUAL CONTROL DISABLED")
                    elif event.key == pygame.K_r:
                        if manual_enabled:
                            reverse_toggle = not reverse_toggle

            if call_exit:
                break

            if manual_enabled and is_alive(ego_vehicle):
                keys = pygame.key.get_pressed()
                ctrl, steer_cache = build_vehicle_control(keys, steer_cache, reverse_toggle, MANUAL_CFG)
                ego_vehicle.apply_control(ctrl)

            if follow_enabled and spectator is not None and is_alive(ego_vehicle):
                mode = SPECTATOR_MODES[cam_mode_idx]
                spectator.set_transform(spectator_follow_tf(ego_vehicle.get_transform(), mode, SPEC_CFG))

            screen.fill((0, 0, 0))
            if surface is not None:
                screen.blit(surface, (0, 0))

            hud = [
                "Sensor: RADAR",
                f"Mode:{RADAR_MODE} | range={radar_range_m:.1f}m",
                f"C:{SPECTATOR_MODES[cam_mode_idx]} | V:{'FOLLOW' if follow_enabled else 'MAP_TOP'} | Q:{'MANUAL' if manual_enabled else 'AUTO'} | R:{'ON' if reverse_toggle else 'OFF'}",
                f"WORLD sync={world.get_settings().synchronous_mode}, dt={world.get_settings().fixed_delta_seconds}",
            ]
            y = 8
            for line in hud:
                screen.blit(font.render(line, True, (240, 240, 240)), (10, y))
                y += 22

            pygame.display.flip()

    finally:
        try:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        except Exception:
            pass

        try:
            if actor_ids:
                client.apply_batch([carla.command.DestroyActor(x) for x in actor_ids])
        except Exception:
            pass

        try:
            world.apply_settings(original_settings)
        except Exception:
            pass

        try:
            pygame.quit()
        except Exception:
            pass


if __name__ == "__main__":
    main()