#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_5_attributeinitester.py
# Description: trafficmanager.ini 의 Traffic Manager 설정을 실행 중에 바꿔 보며 효과 확인
# 사용법:
#   1) 실행하면 차량·보행자가 생성되고 카메라가 차를 따라감
#   2) 실행 중 같은 폴더의 trafficmanager.ini 를 고쳐 저장 -> 1초마다 다시 읽어 반영 ([INI UPDATER] 출력)
#   3) 키: T 차량/보행자 시점 전환, Z/X 이전/다음 대상, C 카메라 모드, V 자유 시점/따라가기,
#          Q 수동 조작 켜고 끄기(WASD, Space, R 후진), ESC 종료
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  11, 2026: Seokhwan Jeong - Created.
#      Feb  11, 2026: Seokhwan Jeong - Added Human spawn and control.
#      Feb  12, 2026: Seokhwan Jeong - Added force_lane_change.
#======================================================================#

import carla
import random
import pygame
import time
import os
import math
import configparser

#======================================================================#
#- Configurations
#======================================================================#
#- CARLA connection
CARLA_HOST = "localhost"
CARLA_PORT = 2000
CLIENT_TIMEOUT_SEC = 5.0

#- World sync mode
SYNC_MODE = True
FIXED_DELTA_SECONDS = 0.05

#- Actor seeds
PY_RANDOM_SEED = 0
TM_RANDOM_SEED = 0

#- Traffic Manager config file
CFG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trafficmanager.ini")   # 스크립트 옆 ini (실행 위치 무관)
CFG_DEFAULT_RELOAD_SEC = 1.0

#- Spawn counts
NUM_VEHICLES = 50
NUM_WALKERS  = 80

#- Vehicle selection
USE_MODEL_FILTER = False
VEHICLE_MODEL_FILTER = [
    "dodge", "audi", "model3", "mini", "mustang", "lincoln",
    "prius", "nissan", "crown", "impala"
]
RANDOMIZE_VEHICLE_COLOR = True
RANDOMIZE_DRIVER_ID = True

#- Walker behavior
WALKER_SPAWN_TRIES = 400
WALKER_CROSS_FACTOR = 0.1          # 0.0~1.0
WALKER_SPEED_MIN = 1.0             # m/s
WALKER_SPEED_MAX = 2.2             # m/s

WALKER_DEST_MIN_DIST = 10.0
WALKER_DEST_TRIES = 40
WALKER_RETARGET_SEC = 10.0

WALKER_STUCK_CHECK_SEC = 1.0
WALKER_STUCK_DIST_EPS = 0.20       # m
WALKER_STUCK_SEC = 6.0             
WALKER_STUCK_RETARGET_TRIES = 60   

USE_KINEMATIC_WALKER_MOVE = True

UI_SIZE = (820, 230)
UI_CAPTION = "Z/X Prev/Next | T Switch(Veh/Walker) | C Cam | V MapFree | Q Manual(WASD) | R Reverse(veh) | ESC/Ctrl+Q Quit"
UI_FONT_NAME = "Arial"
UI_FONT_SIZE = 18

CAM_MODES = ["CHASE", "TOPVIEW_FOLLOW", "FIRST_PERSON"]

# Vehicle cam
V_CHASE_BACK = 8.0
V_CHASE_UP = 3.0
V_CHASE_PITCH = -10.0

V_TOP_Z = 50.0
V_TOP_PITCH = -89.0
V_TOP_YAW = 0.0

V_FP_FRONT = 1.8
V_FP_UP = 1.35
V_FP_PITCH = -2.0

# Walker cam
W_CHASE_BACK = 3.5
W_CHASE_UP = 1.8
W_CHASE_PITCH = -12.0

W_TOP_Z = 25.0
W_TOP_PITCH = -89.0
W_TOP_YAW = 0.0

W_FP_FRONT = 0.5
W_FP_UP = 1.6
W_FP_PITCH = -5.0

# Map view
MAP_VIEW_Z = 220.0
MAP_VIEW_PITCH = -89.0
MAP_VIEW_YAW = -90.0

#- Manual driving input (vehicle)
MANUAL_THROTTLE = 0.70
MANUAL_BRAKE = 1.00
STEER_STEP = 0.03
STEER_RETURN = 0.80

#- Manual control (walker)
WALKER_MANUAL_SPEED = 1.8          # m/s
WALKER_RUN_MULT = 1.6              
WALKER_JUMP = True                 
WALKER_MANUAL_YAW_RATE_DPS = 90.0  # deg/sec

#======================================================================#
#- Utility
#======================================================================#

def is_alive(actor):
    try:
        return actor is not None and actor.is_alive
    except Exception:
        return False

def safe_destroy_actors(client, actors):
    to_destroy = [a for a in actors if is_alive(a)]
    if to_destroy:
        client.apply_batch([carla.command.DestroyActor(a) for a in to_destroy])

def compute_map_center_from_spawnpoints(spawn_points):
    if not spawn_points:
        return carla.Location(0.0, 0.0, 0.0)
    sx = sum(sp.location.x for sp in spawn_points)
    sy = sum(sp.location.y for sp in spawn_points)
    sz = sum(sp.location.z for sp in spawn_points)
    n = float(len(spawn_points))
    return carla.Location(x=sx/n, y=sy/n, z=sz/n)

def select_next_alive(actors, current_idx, step):
    n = len(actors)
    if n == 0:
        return 0, None
    for _ in range(n):
        current_idx = (current_idx + step) % n
        if is_alive(actors[current_idx]):
            return current_idx, actors[current_idx]
    return 0, None

def actor_desc(actor):
    if not is_alive(actor):
        return "N/A"
    attrs = getattr(actor, "attributes", {}) or {}
    color = attrs.get("color", "")
    role = attrs.get("role_name", "")
    name = actor.type_id
    extra = []
    if color:
        extra.append(f"color={color}")
    if role:
        extra.append(f"role={role}")
    extra_s = (" | " + ", ".join(extra)) if extra else ""
    return f"id={actor.id} | {name}{extra_s}"

def _wrap_angle_deg(a):
    # [-180, 180)
    a = (a + 180.0) % 360.0 - 180.0
    return a

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _clamp_pct(v):
    try:
        v = float(v)
    except Exception:
        return 0.0
    return max(0.0, min(100.0, v))

def load_tm_config_ini(path: str):
    cfg = configparser.ConfigParser()
    read_ok = cfg.read(path, encoding="utf-8")
    if not read_ok:
        return None

    ap_enabled = cfg.getboolean("autopilot", "enabled", fallback=True)

    tm = {
        "auto_lane_change": cfg.getboolean("traffic_manager", "auto_lane_change", fallback=False),
        "distance_to_leading_vehicle": cfg.getfloat("traffic_manager", "distance_to_leading_vehicle", fallback=4.0),

        "ignore_lights_percentage": _clamp_pct(cfg.get("traffic_manager", "ignore_lights_percentage", fallback="0")),
        "ignore_signs_percentage": _clamp_pct(cfg.get("traffic_manager", "ignore_signs_percentage", fallback="0")),
        "ignore_vehicles_percentage": _clamp_pct(cfg.get("traffic_manager", "ignore_vehicles_percentage", fallback="0")),
        "ignore_walkers_percentage": _clamp_pct(cfg.get("traffic_manager", "ignore_walkers_percentage", fallback="0")),

        "keep_slow_lane_rule_percentage": _clamp_pct(cfg.get("traffic_manager", "keep_slow_lane_rule_percentage", fallback="100")),
        "random_left_lanechange_percentage": _clamp_pct(cfg.get("traffic_manager", "random_left_lanechange_percentage", fallback="0")),
        "random_right_lanechange_percentage": _clamp_pct(cfg.get("traffic_manager", "random_right_lanechange_percentage", fallback="0")),

        "update_vehicle_lights": cfg.getboolean("traffic_manager", "update_vehicle_lights", fallback=True),
        "vehicle_lane_offset": cfg.getfloat("traffic_manager", "vehicle_lane_offset", fallback=0.0),
        "vehicle_percentage_speed_difference": cfg.getfloat("traffic_manager", "vehicle_percentage_speed_difference", fallback=0.0),

        "force_lane_change_enabled": cfg.getboolean("traffic_manager", "force_lane_change_enabled", fallback=False),
        "force_lane_change_direction": cfg.getboolean("traffic_manager", "force_lane_change_direction", fallback=True),
    }

    interval_sec = cfg.getfloat("reload", "interval_sec", fallback=CFG_DEFAULT_RELOAD_SEC)
    interval_sec = max(0.1, float(interval_sec))
    return ap_enabled, tm, interval_sec

def apply_tm_to_all(vehicles, traffic_manager, tm_port, ap_enabled, tm_cfg,
                    manual_enabled=False, manual_vehicle=None):
    for v in vehicles:
        if not is_alive(v):
            continue
        if manual_enabled and (v == manual_vehicle):
            continue

        v.set_autopilot(bool(ap_enabled), tm_port)

        traffic_manager.auto_lane_change(v, bool(tm_cfg["auto_lane_change"]))
        traffic_manager.distance_to_leading_vehicle(v, float(tm_cfg["distance_to_leading_vehicle"]))

        traffic_manager.ignore_lights_percentage(v, float(tm_cfg["ignore_lights_percentage"]))
        traffic_manager.ignore_signs_percentage(v, float(tm_cfg["ignore_signs_percentage"]))
        traffic_manager.ignore_vehicles_percentage(v, float(tm_cfg["ignore_vehicles_percentage"]))
        traffic_manager.ignore_walkers_percentage(v, float(tm_cfg["ignore_walkers_percentage"]))

        traffic_manager.keep_slow_lane_rule_percentage(v, float(tm_cfg["keep_slow_lane_rule_percentage"]))
        traffic_manager.random_left_lanechange_percentage(v, float(tm_cfg["random_left_lanechange_percentage"]))
        traffic_manager.random_right_lanechange_percentage(v, float(tm_cfg["random_right_lanechange_percentage"]))

        traffic_manager.update_vehicle_lights(v, bool(tm_cfg["update_vehicle_lights"]))
        traffic_manager.vehicle_lane_offset(v, float(tm_cfg["vehicle_lane_offset"]))
        traffic_manager.vehicle_percentage_speed_difference(v, float(tm_cfg["vehicle_percentage_speed_difference"]))

#======================================================================#
#- Spawn Vehicles
#======================================================================#

def get_vehicle_blueprints(world):
    bps = world.get_blueprint_library().filter("*vehicle*")
    if not USE_MODEL_FILTER:
        return list(bps)

    selected = []
    for bp in bps:
        if any(m in bp.id for m in VEHICLE_MODEL_FILTER):
            selected.append(bp)

    return selected if selected else list(bps)

def randomize_vehicle_bp(bp):
    if RANDOMIZE_VEHICLE_COLOR and bp.has_attribute("color"):
        bp.set_attribute("color", random.choice(bp.get_attribute("color").recommended_values))
    if RANDOMIZE_DRIVER_ID and bp.has_attribute("driver_id"):
        bp.set_attribute("driver_id", random.choice(bp.get_attribute("driver_id").recommended_values))

def spawn_vehicles(world, spawn_points, count):
    vehicles = []
    blueprints = get_vehicle_blueprints(world)
    if not blueprints:
        print("[TRAFFIC MANAGER] No vehicle blueprints found.")
        return vehicles

    n = min(count, len(spawn_points))
    for sp in random.sample(spawn_points, n):
        bp = random.choice(blueprints)
        randomize_vehicle_bp(bp)
        actor = world.try_spawn_actor(bp, sp)
        if actor is not None:
            vehicles.append(actor)

    print(f"[TRAFFIC MANAGER] Spawned {len(vehicles)} vehicles")
    return vehicles

def pick_nav_location(world, tries=20):
    for _ in range(max(1, tries)):
        loc = world.get_random_location_from_navigation()
        if loc is not None:
            return loc
    return None

def pick_nav_location_far(world, origin_loc=None, min_dist=10.0, tries=40):
    for _ in range(max(1, tries)):
        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue
        if origin_loc is None:
            return loc
        try:
            if loc.distance(origin_loc) >= float(min_dist):
                return loc
        except Exception:
            continue
    return None

def spawn_walkers(world, num_walkers):
    walkers = []
    walker_ctrl_map = {}
    bp_lib = world.get_blueprint_library()
    walker_bps = bp_lib.filter("walker.pedestrian.*")
    controller_bp = bp_lib.find("controller.ai.walker")

    if not walker_bps:
        print("[TRAFFIC MANAGER] No walker blueprints found.")
        return walkers, walker_ctrl_map, controller_bp

    try:
        world.set_pedestrians_cross_factor(float(WALKER_CROSS_FACTOR))
    except Exception:
        pass

    spawned = 0
    tries = 0
    while spawned < num_walkers and tries < WALKER_SPAWN_TRIES:
        tries += 1
        loc = pick_nav_location(world, tries=10)
        if loc is None:
            continue

        wb = random.choice(walker_bps)
        if wb.has_attribute("is_invincible"):
            wb.set_attribute("is_invincible", "true")

        w = world.try_spawn_actor(wb, carla.Transform(loc))
        if w is None:
            continue

        walkers.append(w)
        spawned += 1

    for w in walkers:
        c = world.try_spawn_actor(controller_bp, carla.Transform(), attach_to=w)
        if c is not None:
            walker_ctrl_map[w.id] = c

    world.tick()

    for w in walkers:
        c = walker_ctrl_map.get(w.id, None)
        if not (is_alive(w) and is_alive(c)):
            continue
        try:
            c.start()
        except Exception:
            continue

        origin = w.get_location()
        dest = pick_nav_location_far(world, origin_loc=origin, min_dist=WALKER_DEST_MIN_DIST, tries=WALKER_DEST_TRIES)
        if dest is None:
            dest = pick_nav_location(world, tries=20)

        if dest is not None:
            try:
                c.go_to_location(dest)
            except Exception:
                pass

        speed = random.uniform(WALKER_SPEED_MIN, WALKER_SPEED_MAX)
        try:
            c.set_max_speed(float(speed))
        except Exception:
            pass

    print(f"[TRAFFIC MANAGER] Spawned {len(walkers)} walkers")
    return walkers, walker_ctrl_map, controller_bp

def destroy_walker_controller(world, walker_ctrl_map, walker_id):
    c = walker_ctrl_map.get(walker_id, None)
    if is_alive(c):
        try:
            c.stop()
        except Exception:
            pass
        try:
            c.destroy()
        except Exception:
            pass
    if walker_id in walker_ctrl_map:
        walker_ctrl_map.pop(walker_id, None)
    world.tick()

def ensure_walker_controller(world, controller_bp, walker, walker_ctrl_map):
    if not is_alive(walker):
        return None
    c = walker_ctrl_map.get(walker.id, None)
    if is_alive(c):
        return c

    c = world.try_spawn_actor(controller_bp, carla.Transform(), attach_to=walker)
    if c is not None:
        walker_ctrl_map[walker.id] = c
        world.tick()
        try:
            c.start()
        except Exception:
            pass
    return c

def retarget_walkers(world, walkers, walker_ctrl_map, exclude_walker_id=None):
    for w in walkers:
        if not is_alive(w):
            continue
        if exclude_walker_id is not None and w.id == exclude_walker_id:
            continue
        c = walker_ctrl_map.get(w.id, None)
        if not is_alive(c):
            continue

        origin = w.get_location()
        dest = pick_nav_location_far(world, origin_loc=origin, min_dist=WALKER_DEST_MIN_DIST, tries=WALKER_DEST_TRIES)
        if dest is None:
            dest = pick_nav_location(world, tries=20)
        if dest is None:
            continue

        try:
            c.go_to_location(dest)
        except Exception:
            pass

def stop_and_destroy_walkers(client, walkers, walker_ctrl_map):
    for _, c in list(walker_ctrl_map.items()):
        if is_alive(c):
            try:
                c.stop()
            except Exception:
                pass
    safe_destroy_actors(client, list(walker_ctrl_map.values()))
    safe_destroy_actors(client, list(walkers))
    walker_ctrl_map.clear()

def build_vehicle_control(keys, steer_cache, reverse_toggle):
    if keys[pygame.K_a]:
        steer_cache -= STEER_STEP
    elif keys[pygame.K_d]:
        steer_cache += STEER_STEP
    else:
        steer_cache *= STEER_RETURN
    steer_cache = max(-1.0, min(1.0, steer_cache))

    throttle = 0.0
    brake = 0.0
    if keys[pygame.K_w] and not keys[pygame.K_s]:
        throttle = MANUAL_THROTTLE
    elif keys[pygame.K_s] and not keys[pygame.K_w]:
        brake = MANUAL_BRAKE

    hand_brake = bool(keys[pygame.K_SPACE])

    return carla.VehicleControl(
        throttle=float(throttle),
        steer=float(steer_cache),
        brake=float(brake),
        hand_brake=hand_brake,
        reverse=bool(reverse_toggle)
    ), steer_cache

def build_walker_control(keys, base_speed, dt, current_tf):
    forward = current_tf.get_forward_vector()
    right = current_tf.get_right_vector()

    dir_x = 0.0
    dir_y = 0.0

    if keys[pygame.K_w]:
        dir_x += forward.x
        dir_y += forward.y
    if keys[pygame.K_s]:
        dir_x -= forward.x
        dir_y -= forward.y
    if keys[pygame.K_d]:
        dir_x += right.x
        dir_y += right.y
    if keys[pygame.K_a]:
        dir_x -= right.x
        dir_y -= right.y

    mag = math.sqrt(dir_x*dir_x + dir_y*dir_y)
    speed = 0.0
    direction = carla.Vector3D(0.0, 0.0, 0.0)

    if mag > 1e-6:
        direction = carla.Vector3D(dir_x/mag, dir_y/mag, 0.0)
        speed = float(base_speed)
        if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
            speed *= WALKER_RUN_MULT

    jump = bool(WALKER_JUMP and keys[pygame.K_SPACE])
    ctrl = carla.WalkerControl(direction=direction, speed=speed, jump=jump)

    moved_tf = None
    if USE_KINEMATIC_WALKER_MOVE and speed > 0.0:
        moved_tf = carla.Transform(
            carla.Location(
                x=current_tf.location.x + direction.x * speed * dt,
                y=current_tf.location.y + direction.y * speed * dt,
                z=current_tf.location.z
            ),
            current_tf.rotation
        )
        target_yaw = math.degrees(math.atan2(direction.y, direction.x))
        cur_yaw = float(current_tf.rotation.yaw)
        dyaw = _wrap_angle_deg(target_yaw - cur_yaw)

        max_step = float(WALKER_MANUAL_YAW_RATE_DPS) * float(dt)
        dyaw = _clamp(dyaw, -max_step, +max_step)
        moved_tf.rotation.yaw = cur_yaw + dyaw

    return ctrl, moved_tf

def spectator_tf_follow(actor_tf, kind, mode):
    t = actor_tf

    if kind == "vehicle":
        if mode == "CHASE":
            offset = carla.Location(x=-V_CHASE_BACK, y=0.0, z=V_CHASE_UP)
            cam_loc = t.transform(offset)
            cam_rot = carla.Rotation(pitch=V_CHASE_PITCH, yaw=t.rotation.yaw, roll=0.0)
            return carla.Transform(cam_loc, cam_rot)

        if mode == "TOPVIEW_FOLLOW":
            loc = carla.Location(x=t.location.x, y=t.location.y, z=t.location.z + V_TOP_Z)
            rot = carla.Rotation(pitch=V_TOP_PITCH, yaw=V_TOP_YAW, roll=0.0)
            return carla.Transform(loc, rot)

        offset = carla.Location(x=V_FP_FRONT, y=0.0, z=V_FP_UP)
        cam_loc = t.transform(offset)
        cam_rot = carla.Rotation(pitch=V_FP_PITCH, yaw=t.rotation.yaw, roll=0.0)
        return carla.Transform(cam_loc, cam_rot)

    if mode == "CHASE":
        offset = carla.Location(x=-W_CHASE_BACK, y=0.0, z=W_CHASE_UP)
        cam_loc = t.transform(offset)
        cam_rot = carla.Rotation(pitch=W_CHASE_PITCH, yaw=t.rotation.yaw, roll=0.0)
        return carla.Transform(cam_loc, cam_rot)

    if mode == "TOPVIEW_FOLLOW":
        loc = carla.Location(x=t.location.x, y=t.location.y, z=t.location.z + W_TOP_Z)
        rot = carla.Rotation(pitch=W_TOP_PITCH, yaw=W_TOP_YAW, roll=0.0)
        return carla.Transform(loc, rot)

    offset = carla.Location(x=W_FP_FRONT, y=0.0, z=W_FP_UP)
    cam_loc = t.transform(offset)
    cam_rot = carla.Rotation(pitch=W_FP_PITCH, yaw=t.rotation.yaw, roll=0.0)
    return carla.Transform(cam_loc, cam_rot)

#======================================================================#
#- Main
#======================================================================#

def main():
    # pygame
    pygame.init()
    screen = pygame.display.set_mode(UI_SIZE)
    pygame.display.set_caption(UI_CAPTION)
    font = pygame.font.SysFont(UI_FONT_NAME, UI_FONT_SIZE)

    # carla
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(CLIENT_TIMEOUT_SEC)

    world = client.get_world()
    original_settings = world.get_settings()

    vehicles = []
    walkers = []
    walker_ctrl_map = {}
    controller_bp = None

    spectator = None
    map_view_tf = None

    # Follow state
    follow_kind = "vehicle"
    v_idx = 0
    w_idx = 0
    follow_actor = None

    # Cam state
    cam_mode_idx = 0
    follow_enabled = True

    # Manual state
    manual_enabled = False
    reverse_toggle = False
    steer_cache = 0.0

    # TM state
    ap_enabled_current = True
    tm_cfg_current = None
    reload_interval = CFG_DEFAULT_RELOAD_SEC
    last_mtime = None
    next_ini_check = 0.0

    force_last_enabled = False
    force_pending = False
    force_dir_cached = True

    next_walker_retarget = 0.0
    next_walker_stuck_check = 0.0
    walker_last_loc = {}
    walker_last_move_t = {}

    try:
        # seeds
        if PY_RANDOM_SEED is not None:
            random.seed(PY_RANDOM_SEED)

        # sync
        settings = world.get_settings()
        settings.synchronous_mode = bool(SYNC_MODE)
        settings.fixed_delta_seconds = float(FIXED_DELTA_SECONDS) if SYNC_MODE else None
        world.apply_settings(settings)

        # TM
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(bool(SYNC_MODE))
        tm_port = traffic_manager.get_port()
        if TM_RANDOM_SEED is not None:
            traffic_manager.set_random_device_seed(int(TM_RANDOM_SEED))

        spectator = world.get_spectator()
        spawn_points = world.get_map().get_spawn_points()

        # map view tf
        map_center = compute_map_center_from_spawnpoints(spawn_points)
        map_view_tf = carla.Transform(
            carla.Location(x=map_center.x, y=map_center.y, z=map_center.z + MAP_VIEW_Z),
            carla.Rotation(pitch=MAP_VIEW_PITCH, yaw=MAP_VIEW_YAW, roll=0.0)
        )

        # spawn
        vehicles = spawn_vehicles(world, spawn_points, NUM_VEHICLES)
        if not vehicles:
            print("[TRAFFIC MANAGER] No vehicles spawned.")
            return

        walkers, walker_ctrl_map, controller_bp = spawn_walkers(world, NUM_WALKERS)

        # init walker tracking
        now0 = time.time()
        for w in walkers:
            if is_alive(w):
                walker_last_loc[w.id] = w.get_location()
                walker_last_move_t[w.id] = now0

        next_walker_retarget = now0 + WALKER_RETARGET_SEC
        next_walker_stuck_check = now0 + WALKER_STUCK_CHECK_SEC

        # TM defaults
        default_tm = {
            "auto_lane_change": False,
            "distance_to_leading_vehicle": 4.0,
            "ignore_lights_percentage": 0.0,
            "ignore_signs_percentage": 0.0,
            "ignore_vehicles_percentage": 0.0,
            "ignore_walkers_percentage": 0.0,
            "keep_slow_lane_rule_percentage": 100.0,
            "random_left_lanechange_percentage": 0.0,
            "random_right_lanechange_percentage": 0.0,
            "update_vehicle_lights": True,
            "vehicle_lane_offset": 0.0,
            "vehicle_percentage_speed_difference": 0.0,
            "force_lane_change_enabled": False,
            "force_lane_change_direction": True,
        }
        tm_cfg_current = default_tm
        ap_enabled_current = True

        # ini initial load
        state = load_tm_config_ini(CFG_PATH)
        if state is not None:
            ap_enabled_current, tm_cfg_current, reload_interval = state
            try:
                last_mtime = os.path.getmtime(CFG_PATH)
            except Exception:
                last_mtime = None
            print("[INI UPDATER] Applied ini configuration")
        else:
            reload_interval = CFG_DEFAULT_RELOAD_SEC
            print("[INI UPDATER] ini not found/unreadable, applying default TM config")

        apply_tm_to_all(vehicles, traffic_manager, tm_port, ap_enabled_current, tm_cfg_current)

        v_idx, follow_actor = select_next_alive(vehicles, -1, +1)
        follow_kind = "vehicle"
        if follow_actor is None:
            print("[INI UPDATER] No alive vehicles after spawn.")
            return

        force_last_enabled = bool(tm_cfg_current.get("force_lane_change_enabled", False))
        force_pending = False
        force_dir_cached = bool(tm_cfg_current.get("force_lane_change_direction", True))

        next_ini_check = time.time() + float(reload_interval)

        running = True
        while running:
            if SYNC_MODE:
                world.tick()
                dt = float(FIXED_DELTA_SECONDS)
            else:
                world.wait_for_tick()
                snap = world.get_snapshot()
                dt = float(snap.timestamp.delta_seconds) if snap else 0.05

            now = time.time()

            if walkers and now >= next_walker_retarget:
                exclude_id = follow_actor.id if (manual_enabled and follow_kind == "walker" and is_alive(follow_actor)) else None
                retarget_walkers(world, walkers, walker_ctrl_map, exclude_walker_id=exclude_id)
                next_walker_retarget = now + WALKER_RETARGET_SEC

            if walkers and now >= next_walker_stuck_check:
                exclude_id = follow_actor.id if (manual_enabled and follow_kind == "walker" and is_alive(follow_actor)) else None

                for w in walkers:
                    if not is_alive(w):
                        continue
                    if exclude_id is not None and w.id == exclude_id:
                        continue

                    loc = w.get_location()
                    prev = walker_last_loc.get(w.id, None)
                    if prev is None:
                        walker_last_loc[w.id] = loc
                        walker_last_move_t[w.id] = now
                        continue

                    dist = loc.distance(prev)
                    if dist >= WALKER_STUCK_DIST_EPS:
                        walker_last_loc[w.id] = loc
                        walker_last_move_t[w.id] = now
                    else:
                        last_move = walker_last_move_t.get(w.id, now)
                        if (now - last_move) >= WALKER_STUCK_SEC:
                            c = walker_ctrl_map.get(w.id, None)
                            if is_alive(c):
                                origin = w.get_location()
                                dest = pick_nav_location_far(
                                    world, origin_loc=origin,
                                    min_dist=WALKER_DEST_MIN_DIST,
                                    tries=WALKER_STUCK_RETARGET_TRIES
                                )
                                if dest is None:
                                    dest = pick_nav_location(world, tries=20)

                                if dest is not None:
                                    try:
                                        c.go_to_location(dest)
                                        walker_last_move_t[w.id] = now
                                    except Exception:
                                        pass

                next_walker_stuck_check = now + WALKER_STUCK_CHECK_SEC

            if now >= next_ini_check:
                try:
                    mtime = os.path.getmtime(CFG_PATH)
                except Exception:
                    mtime = None

                if mtime is not None and mtime != last_mtime:
                    state = load_tm_config_ini(CFG_PATH)
                    if state is not None:
                        ap_enabled_current, tm_cfg_current, reload_interval = state
                        apply_tm_to_all(
                            vehicles, traffic_manager, tm_port,
                            ap_enabled_current, tm_cfg_current,
                            manual_enabled=(manual_enabled and follow_kind == "vehicle"),
                            manual_vehicle=(follow_actor if follow_kind == "vehicle" else None)
                        )
                        if manual_enabled and follow_kind == "vehicle" and is_alive(follow_actor):
                            follow_actor.set_autopilot(False, tm_port)

                        print(f"[INI UPDATER] Reloaded new INI file.")
                        last_mtime = mtime
                    else:
                        print("[WARN] ini changed but parse failed. Keeping previous settings.")

                next_ini_check = now + float(reload_interval)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    mods = pygame.key.get_mods()

                    if event.key == pygame.K_ESCAPE or (event.key == pygame.K_q and (mods & pygame.KMOD_CTRL)):
                        running = False

                    elif event.key == pygame.K_t:
                        if manual_enabled:
                            manual_enabled = False
                            reverse_toggle = False
                            steer_cache = 0.0
                            if follow_kind == "vehicle" and is_alive(follow_actor):
                                follow_actor.set_autopilot(bool(ap_enabled_current), tm_port)
                            elif follow_kind == "walker" and is_alive(follow_actor):
                                if controller_bp is not None:
                                    ensure_walker_controller(world, controller_bp, follow_actor, walker_ctrl_map)

                        if follow_kind == "vehicle":
                            follow_kind = "walker"
                            w_idx, follow_actor = select_next_alive(walkers, w_idx - 1, +1)
                            if follow_actor is None:
                                follow_kind = "vehicle"
                                v_idx, follow_actor = select_next_alive(vehicles, v_idx - 1, +1)
                                print("[WARN] No alive walkers")
                            else:
                                print("[VIEW MODE] Changed to WALKER Mode")
                        else:
                            follow_kind = "vehicle"
                            v_idx, follow_actor = select_next_alive(vehicles, v_idx - 1, +1)
                            if follow_actor is None:
                                follow_kind = "walker"
                                w_idx, follow_actor = select_next_alive(walkers, w_idx - 1, +1)
                                print("[VIEW MODE] No alive vehicles")
                            else:
                                print("[VIEW MODE] Changed to VEHICLE Mode")

                    elif event.key == pygame.K_z:
                        if follow_kind == "vehicle":
                            v_idx, follow_actor = select_next_alive(vehicles, v_idx, -1)
                        else:
                            w_idx, follow_actor = select_next_alive(walkers, w_idx, -1)
                        reverse_toggle = False
                        steer_cache = 0.0
                        manual_enabled = False
                        if follow_kind == "vehicle" and is_alive(follow_actor):
                            follow_actor.set_autopilot(bool(ap_enabled_current), tm_port)
                        elif follow_kind == "walker" and is_alive(follow_actor) and controller_bp is not None:
                            ensure_walker_controller(world, controller_bp, follow_actor, walker_ctrl_map)
                        print("[VIEW MODE] Changing to Previous Actor")

                    elif event.key == pygame.K_x:
                        if follow_kind == "vehicle":
                            v_idx, follow_actor = select_next_alive(vehicles, v_idx, +1)
                        else:
                            w_idx, follow_actor = select_next_alive(walkers, w_idx, +1)
                        reverse_toggle = False
                        steer_cache = 0.0
                        manual_enabled = False
                        if follow_kind == "vehicle" and is_alive(follow_actor):
                            follow_actor.set_autopilot(bool(ap_enabled_current), tm_port)
                        elif follow_kind == "walker" and is_alive(follow_actor) and controller_bp is not None:
                            ensure_walker_controller(world, controller_bp, follow_actor, walker_ctrl_map)
                        print("[VIEW MODE] Changing to Next Actor")

                    elif event.key == pygame.K_c:
                        cam_mode_idx = (cam_mode_idx + 1) % len(CAM_MODES)
                        print(f"[VIEW MODE] Camera Mode: {CAM_MODES[cam_mode_idx]}")

                    elif event.key == pygame.K_v:
                        follow_enabled = not follow_enabled
                        if not follow_enabled:
                            spectator.set_transform(map_view_tf)
                            print("[VIEW MODE] Camera Mode: Free View")
                        else:
                            print("[VIEW MODE] Camera Mode: Follow Actor")

                    elif event.key == pygame.K_q:
                        manual_enabled = not manual_enabled
                        reverse_toggle = False
                        steer_cache = 0.0

                        if manual_enabled:
                            if not is_alive(follow_actor):
                                manual_enabled = False
                                print("[WARN] MANUAL requested but follow actor is dead.")
                            else:
                                if follow_kind == "vehicle":
                                    follow_actor.set_autopilot(False, tm_port)
                                    print("[AUTOPILOT] OFF, MANUAL DRIVING ENABLED")
                                else:
                                    destroy_walker_controller(world, walker_ctrl_map, follow_actor.id)
                                    print("[AUTOPILOT] OFF, MANUAL CONTROL ENABLED")
                        else:
                            if is_alive(follow_actor):
                                if follow_kind == "vehicle":
                                    follow_actor.set_autopilot(bool(ap_enabled_current), tm_port)
                                    print("[AUTOPILOT] ON, MANUAL DRIVING DISABLED")
                                else:
                                    if controller_bp is not None:
                                        ensure_walker_controller(world, controller_bp, follow_actor, walker_ctrl_map)
                                    print("[AUTOPILOT] ON, MANUAL CONTROL DISABLED")

                    elif event.key == pygame.K_r:
                        if manual_enabled and follow_kind == "vehicle":
                            reverse_toggle = not reverse_toggle

            if not running:
                break

            if (manual_enabled or follow_enabled) and (not is_alive(follow_actor)):
                if follow_kind == "vehicle":
                    v_idx, follow_actor = select_next_alive(vehicles, v_idx, +1)
                    if follow_actor is None:
                        running = False
                        break
                else:
                    w_idx, follow_actor = select_next_alive(walkers, w_idx, +1)
                    if follow_actor is None:
                        follow_kind = "vehicle"
                        v_idx, follow_actor = select_next_alive(vehicles, v_idx, +1)
                        if follow_actor is None:
                            running = False
                            break
                manual_enabled = False
                reverse_toggle = False
                steer_cache = 0.0
                print("[VIEW MODE] Follow target died, switched to available actor")

            try:
                cur_force_enabled = bool(tm_cfg_current.get("force_lane_change_enabled", False))
                force_dir_cached = bool(tm_cfg_current.get("force_lane_change_direction", True))

                if cur_force_enabled and (not force_last_enabled):
                    force_pending = True

                if not cur_force_enabled:
                    force_pending = False

                force_last_enabled = cur_force_enabled

                if force_pending:
                    if follow_kind == "vehicle" and is_alive(follow_actor) and not manual_enabled:
                        traffic_manager.force_lane_change(follow_actor, force_dir_cached)
                        force_pending = False
                        print(f"[FORCE LANE CHANGE] Applied once to id={follow_actor.id} dir={'RIGHT' if force_dir_cached else 'LEFT'}")
            except Exception as e:
                print(f"[FORCE LANE CHANGE] Failed: {e}")

            if manual_enabled and is_alive(follow_actor):
                keys = pygame.key.get_pressed()
                if follow_kind == "vehicle":
                    ctrl, steer_cache = build_vehicle_control(keys, steer_cache, reverse_toggle)
                    follow_actor.apply_control(ctrl)
                else:
                    current_tf = follow_actor.get_transform()
                    ctrl, moved_tf = build_walker_control(keys, WALKER_MANUAL_SPEED, dt, current_tf)
                    follow_actor.apply_control(ctrl)
                    if moved_tf is not None:
                        try:
                            follow_actor.set_transform(moved_tf)
                        except Exception:
                            pass

            if follow_enabled and is_alive(follow_actor):
                mode = CAM_MODES[cam_mode_idx]
                tf = spectator_tf_follow(follow_actor.get_transform(), follow_kind, mode)
                spectator.set_transform(tf)

            screen.fill((20, 20, 20))
            follow_line = f"FOLLOW={follow_kind.upper()} | {actor_desc(follow_actor)}"
            lines = [
                "Z/X Prev/Next | T Switch(Veh/Walker) | C Cam | V MapFree | Q Manual(WASD) | R Reverse(veh) | ESC/Ctrl+Q Quit",
                f"FOLLOW_CAM: {'ON' if follow_enabled else 'OFF (FREE MAP VIEW)'} | CamMode={CAM_MODES[cam_mode_idx]}",
                f"DRIVE_MODE: {'MANUAL' if manual_enabled else 'AUTO'} | REVERSE(veh): {'ON' if reverse_toggle else 'OFF'}",
                f"Vehicles={len(vehicles)} | Walkers={len(walkers)} | ReloadINI={reload_interval:.2f}s | DestMin={WALKER_DEST_MIN_DIST:.1f}m",
                f"FORCE_LANE_CHANGE: enabled={tm_cfg_current.get('force_lane_change_enabled', False)} | pending={force_pending} | dir={'RIGHT' if force_dir_cached else 'LEFT'}",
                follow_line
            ]
            y = 10
            for s in lines:
                screen.blit(font.render(s, True, (220, 220, 220)), (10, y))
                y += 24
            pygame.display.flip()

    except KeyboardInterrupt:
        print("\nClosing the simulation.")

    finally:
        try:
            if spectator is not None and map_view_tf is not None:
                spectator.set_transform(map_view_tf)
                if SYNC_MODE:
                    world.tick()
        except Exception:
            pass

        try:
            stop_and_destroy_walkers(client, walkers, walker_ctrl_map)
        except Exception:
            pass
        walkers = []

        safe_destroy_actors(client, vehicles)
        vehicles = []

        try:
            world.apply_settings(original_settings)
        except Exception:
            pass

        print("[INFO] Cleaned up and restored original settings.")

if __name__ == "__main__":
    main()
