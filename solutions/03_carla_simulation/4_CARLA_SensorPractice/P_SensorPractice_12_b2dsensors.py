#!/usr/bin/env python3
#======================================================================#
# Module:      NGV_SensorPractice_12_b2dsensors.py
# Description: Practice for Bench2Drive Sensor setting
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  21, 2026: Seokhwan Jeong - Created.
#======================================================================#
import time
import random
import numpy as np

try:
    import pygame
    from pygame.locals import K_ESCAPE
except ImportError:
    raise RuntimeError("cannot import pygame, make sure pygame is installed")

HAS_SDL2_MULTI_WINDOW = False
try:
    from pygame._sdl2 import Window as SDL2Window
    from pygame._sdl2 import Renderer as SDL2Renderer
    from pygame._sdl2 import Texture as SDL2Texture
    HAS_SDL2_MULTI_WINDOW = True
except Exception:
    HAS_SDL2_MULTI_WINDOW = False

import carla


# ============================================================
# 기본 설정값들
# ============================================================
MAIN_W, MAIN_H = 1200, 450
GRID_COLS = 3
GRID_ROWS = 2

GNSS_W, GNSS_H = 500, 500

NUM_NPC = 20

TERM_PRINT_INTERVAL_SEC = 0.5

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

MAP_WP_STEP = 2.0
MAP_DRAW_STRIDE = 3
MAP_MARGIN_PX = 30
MAP_BG_COLOR = (0, 0, 0)
MAP_WP_COLOR = (255, 255, 255)
MAP_WP_RADIUS = 3
EGO_DOT_COLOR = (255, 0, 0)
EGO_DOT_RADIUS = 6
EGO_HEADING_LEN_M = 8.0

# visual order
CAM_CONFIGS = [
    ("CAM_FRONT_LEFT",""),
    ("CAM_FRONT",""),
    ("CAM_FRONT_RIGHT",""),
    ("CAM_BACK_RIGHT",""),
    ("CAM_BACK",""),
    ("CAM_BACK_LEFT",""),
]


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

    world.tick()

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

    world.tick()

    return npc_ids


def apply_tm_safety_settings(world, traffic_manager, ego_vehicle, npc_ids):
    all_ids = [ego_vehicle.id] + list(npc_ids)
    for vid in all_ids:
        v = world.get_actor(vid)
        if v is None:
            continue
        traffic_manager.distance_to_leading_vehicle(v, 8.0)
        traffic_manager.auto_lane_change(v, False)


def image_to_surface(image):
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = np.reshape(arr, (image.height, image.width, 4))
    arr = arr[:, :, :3]
    arr = arr[:, :, ::-1]  # BGR->RGB
    return pygame.surfarray.make_surface(arr.swapaxes(0, 1))

def blit_fit_keep_aspect(dst_surf, src_surf, cell_x, cell_y, cell_w, cell_h):
    sw, sh = src_surf.get_width(), src_surf.get_height()
    if sw <= 0 or sh <= 0 or cell_w <= 0 or cell_h <= 0:
        return
    scale = min(cell_w / sw, cell_h / sh)
    tw = max(1, int(sw * scale))
    th = max(1, int(sh * scale))
    scaled = pygame.transform.scale(src_surf, (tw, th))
    ox = cell_x + (cell_w - tw) // 2
    oy = cell_y + (cell_h - th) // 2
    dst_surf.blit(scaled, (ox, oy))


# ============================================================
# GNSS Map helper (world -> screen)
# ============================================================
def prepare_map_cache(world, out_w, out_h):
    m = world.get_map()
    wps = m.generate_waypoints(float(MAP_WP_STEP))
    if not wps:
        return None

    xs = [wp.transform.location.x for wp in wps]
    ys = [wp.transform.location.y for wp in wps]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)

    dx = max(maxx - minx, 1e-6)
    dy = max(maxy - miny, 1e-6)

    usable_w = max(out_w - 2 * MAP_MARGIN_PX, 1)
    usable_h = max(out_h - 2 * MAP_MARGIN_PX, 1)

    scale = min(usable_w / dx, usable_h / dy)

    map_w = dx * scale
    map_h = dy * scale
    off_x = int((usable_w - map_w) * 0.5)
    off_y = int((usable_h - map_h) * 0.5)

    return {
        "wps": wps,
        "minx": minx, "miny": miny,
        "scale": scale,
        "off_x": off_x, "off_y": off_y,
        "out_w": out_w, "out_h": out_h,
    }

def world_to_screen(cache, x, y):
    px = MAP_MARGIN_PX + cache["off_x"] + int((x - cache["minx"]) * cache["scale"])
    py = MAP_MARGIN_PX + cache["off_y"] + int((y - cache["miny"]) * cache["scale"])
    # py = cache["out_h"] - py
    return px, py

def build_base_map_surface(cache):
    surf = pygame.Surface((cache["out_w"], cache["out_h"]))
    surf.fill(MAP_BG_COLOR)

    for wp in cache["wps"][::int(MAP_DRAW_STRIDE)]:
        loc = wp.transform.location
        px, py = world_to_screen(cache, loc.x, loc.y)
        if 0 <= px < cache["out_w"] and 0 <= py < cache["out_h"]:
            pygame.draw.circle(surf, MAP_WP_COLOR, (px, py), MAP_WP_RADIUS)
    return surf

class GnssWindowSDL2:
    def __init__(self, title, w, h):
        self.win = SDL2Window(title, size=(w, h))
        self.ren = SDL2Renderer(self.win)
        self.w = w
        self.h = h

    def draw_surface(self, surf):
        tex = SDL2Texture.from_surface(self.ren, surf)

        if hasattr(self.ren, "clear"):
            self.ren.clear()

        if hasattr(tex, "draw"):
            tex.draw(dstrect=(0, 0, self.w, self.h))
        elif hasattr(self.ren, "blit"):
            self.ren.blit(tex, dstrect=(0, 0, self.w, self.h))

        if hasattr(self.ren, "present"):
            self.ren.present()

    def close(self):
        try:
            self.win.destroy()
        except Exception:
            pass


# ============================================================
# Main
# ============================================================
def main():
    pygame.init()

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

    world.tick()

    # ============================================================
    # Traffic Manager를 연결
    # ============================================================
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    tm_port = traffic_manager.get_port()

    actor_ids = []
    ego = None

    spectator = None
    map_view_tf = None

    follow_enabled = True
    cam_mode_idx = 0
    manual_enabled = False
    reverse_toggle = False
    steer_cache = 0.0

    flags = pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE
    screen = pygame.display.set_mode((MAIN_W, MAIN_H), flags)
    pygame.display.set_caption("NGV | 6xRGB (3x2) | C:Cam | V:MapTop | Q:Manual | R:Reverse | ESC Quit")

    font = pygame.font.SysFont("Arial", 18)
    font_small = pygame.font.SysFont("Arial", 16)

    # sensors
    cam_sensors = []
    cam_surfaces = [None] * 6

    gnss_sensor = None
    imu_sensor = None

    last_gnss = None
    last_imu_lines = None
    last_print_t = 0.0

    map_cache = None
    base_map_surface = None

    gnss_window = None
    gnss_fallback_inset = True
    if HAS_SDL2_MULTI_WINDOW:
        try:
            gnss_window = GnssWindowSDL2("NGV | GNSS MAP", GNSS_W, GNSS_H)
            gnss_fallback_inset = False
            print("[GNSS] SDL2 multi-window enabled -> GNSS opens in a new window.")
        except Exception as e:
            gnss_window = None
            gnss_fallback_inset = True
            print("[GNSS] SDL2 window failed -> fallback to inset. err:", e)
    else:
        print("[GNSS] pygame._sdl2 not available -> fallback to inset on main window.")

    try:
        # ============================================================
        # 차량 Spawn (EGO + NPC)
        # ============================================================
        ego, ego_tf, spawn_points = spawn_ego_vehicle(world, tm_port)
        actor_ids.append(ego.id)

        npc_ids = spawn_npc_vehicles(client, world, tm_port, ego_tf)
        actor_ids.extend(npc_ids)

        # ============================================================
        # TM 안전 세팅 적용 (차간거리 설정 등)
        # ============================================================
        apply_tm_safety_settings(world, traffic_manager, ego, npc_ids)

        # ============================================================
        # Spectator / Map view 준비
        # ============================================================
        spectator = world.get_spectator()
        map_center = compute_map_center_from_spawnpoints(spawn_points)
        map_view_tf = map_view_tf_from_cfg(map_center, SPEC_CFG)

        # ============================================================
        # GNSS map cache
        # ============================================================
        map_cache = prepare_map_cache(world, GNSS_W, GNSS_H)
        if map_cache is not None:
            base_map_surface = build_base_map_surface(map_cache)
            print("[MAP] waypoints:", len(map_cache["wps"]))
        else:
            base_map_surface = pygame.Surface((GNSS_W, GNSS_H))
            base_map_surface.fill(MAP_BG_COLOR)
            print("[MAP] map_cache is None -> only background")

        ## ============================================================
        # [TODO] Spawn and attach 6x RGB Cameras
        # ============================================================
        # ----------------------------
        # (1) CAM_FRONT_LEFT
        # ----------------------------
        #-[TODO]- Get rgbcam information from blueprint (use .get_blueprint_library().find())
        #[key functions] get_blueprint_library().find("sensor.????")
        # rgb camera id: "sensor.camera.rgb"
        rgbcam_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")

        #-[TODO]- Set rgbcam attribute (use .set_attribute())
        #[key functions] set_attribute("attribute", "value"))
        rgbcam_blueprint.set_attribute('image_size_x', str(CAM_W))
        rgbcam_blueprint.set_attribute('image_size_y', str(CAM_H))
        rgbcam_blueprint.set_attribute('fov', '90.0')

        #-[TODO]- Set rgbcam transform
        cam_front_left_transform = carla.Transform(carla.Location(x=0.0, y=-1.0, z=2.0), carla.Rotation(roll=0.0, pitch=0.0, yaw=-45.0))
        
        #-[TODO]- Spawn rgbcam actor on ego_vehicle (use .spawn_actor())
        #[key functions] spawn_actor(blueprint, transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_front_left = world.spawn_actor(rgbcam_blueprint, cam_front_left_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_sensors.append(cam_front_left)

        def cam_front_left_cb(image):
            nonlocal cam_surfaces
            image.convert(carla.ColorConverter.Raw)
            cam_surfaces[0] = image_to_surface(image)

        #-[TODO]- Listen to rgbcam data stream (use .listen())
        cam_back_left.listen(cam_back_left_callback)
        cam_back.listen(cam_back_callback)
        cam_back_right.listen(cam_back_right_callback)
        cam_front_right.listen(cam_front_right_callback)
        cam_front.listen(cam_front_callback)
        cam_front_left.listen(cam_front_left_callback)
        #[key functions] use .listen(callback))


        # ----------------------------
        # (2) CAM_FRONT
        # ----------------------------
        #-[TODO]- Get rgbcam information from blueprint (use .get_blueprint_library().find())
        #[key functions] get_blueprint_library().find("sensor.????")
        # rgb camera id: "sensor.camera.rgb"
        rgbcam_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        
        #-[TODO]- Set rgbcam attribute (use .set_attribute())
        #[key functions] set_attribute("attribute", "value"))
        rgbcam_blueprint.set_attribute('image_size_x', str(CAM_W))
        rgbcam_blueprint.set_attribute('image_size_y', str(CAM_H))
        rgbcam_blueprint.set_attribute('fov', '90.0')

        #-[TODO]- Set rgbcam transform
        cam_front_transform = carla.Transform(carla.Location(x=2.0, y=0.0, z=2.0), carla.Rotation(roll=0.0, pitch=0.0, yaw=0.0))

        #-[TODO]- Spawn rgbcam actor on ego_vehicle (use .spawn_actor())
        #[key functions] spawn_actor(blueprint, transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_front = world.spawn_actor(rgbcam_blueprint, cam_front_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_sensors.append(cam_front)

        def cam_front_cb(image):
            nonlocal cam_surfaces
            image.convert(carla.ColorConverter.Raw)
            cam_surfaces[1] = image_to_surface(image)

        #-[TODO]- Listen to rgbcam data stream (use .listen())
        #[key functions] use .listen(callback))


        # ----------------------------
        # (3) CAM_FRONT_RIGHT
        # ----------------------------
        #-[TODO]- Get rgbcam information from blueprint (use .get_blueprint_library().find())
        #[key functions] get_blueprint_library().find("sensor.????")
        # rgb camera id: "sensor.camera.rgb"
        rgbcam_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        
        #-[TODO]- Set rgbcam attribute (use .set_attribute())
        #[key functions] set_attribute("attribute", "value"))
        rgbcam_blueprint.set_attribute('image_size_x', str(CAM_W))
        rgbcam_blueprint.set_attribute('image_size_y', str(CAM_H))
        rgbcam_blueprint.set_attribute('fov', '90.0')

        #-[TODO]- Set rgbcam transform
        cam_front_right_transform = carla.Transform(carla.Location(x=0.0, y=1.0, z=2.0), carla.Rotation(roll=0.0, pitch=0.0, yaw=45.0))

        #-[TODO]- Spawn rgbcam actor on ego_vehicle (use .spawn_actor())
        #[key functions] spawn_actor(blueprint, transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_front_right = world.spawn_actor(rgbcam_blueprint, cam_front_right_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_sensors.append(cam_front_right)

        def cam_front_right_cb(image):
            nonlocal cam_surfaces
            image.convert(carla.ColorConverter.Raw)
            cam_surfaces[2] = image_to_surface(image)

        #-[TODO]- Listen to rgbcam data stream (use .listen())
        #[key functions] use .listen(callback))


        # ----------------------------
        # (4) CAM_BACK_RIGHT
        # ----------------------------
        #-[TODO]- Get rgbcam information from blueprint (use .get_blueprint_library().find())
        #[key functions] get_blueprint_library().find("sensor.????")
        # rgb camera id: "sensor.camera.rgb"
        rgbcam_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")

        #-[TODO]- Set rgbcam attribute (use .set_attribute())
        #[key functions] set_attribute("attribute", "value"))
        rgbcam_blueprint.set_attribute('image_size_x', str(CAM_W))
        rgbcam_blueprint.set_attribute('image_size_y', str(CAM_H))
        rgbcam_blueprint.set_attribute('fov', '90.0')

        #-[TODO]- Set rgbcam transform
        cam_back_right_transform = carla.Transform(carla.Location(x=0.0, y=1.0, z=2.0), carla.Rotation(roll=0.0, pitch=0.0, yaw=135.0))

        #-[TODO]- Spawn rgbcam actor on ego_vehicle (use .spawn_actor())
        #[key functions] spawn_actor(blueprint, transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_back_right = world.spawn_actor(rgbcam_blueprint, cam_back_right_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_sensors.append(cam_back_right)

        def cam_back_right_cb(image):
            nonlocal cam_surfaces
            image.convert(carla.ColorConverter.Raw)
            cam_surfaces[3] = image_to_surface(image)

        #-[TODO]- Listen to rgbcam data stream (use .listen())
        #[key functions] use .listen(callback))

        # ----------------------------
        # (5) CAM_BACK
        # ----------------------------
        #-[TODO]- Get rgbcam information from blueprint (use .get_blueprint_library().find())
        #[key functions] get_blueprint_library().find("sensor.????")
        # rgb camera id: "sensor.camera.rgb"
        rgbcam_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")

        #-[TODO]- Set rgbcam attribute (use .set_attribute())
        #[key functions] set_attribute("attribute", "value"))
        rgbcam_blueprint.set_attribute('image_size_x', str(CAM_W))
        rgbcam_blueprint.set_attribute('image_size_y', str(CAM_H))
        rgbcam_blueprint.set_attribute('fov', '90.0')

        #-[TODO]- Set rgbcam transform
        cam_back_transform = carla.Transform(carla.Location(x=-2.0, y=0.0, z=2.0), carla.Rotation(roll=0.0, pitch=0.0, yaw=180.0))

        #-[TODO]- Spawn rgbcam actor on ego_vehicle (use .spawn_actor())
        #[key functions] spawn_actor(blueprint, transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_back = world.spawn_actor(rgbcam_blueprint, cam_back_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_sensors.append(cam_back)

        def cam_back_cb(image):
            nonlocal cam_surfaces
            image.convert(carla.ColorConverter.Raw)
            cam_surfaces[4] = image_to_surface(image)

        #-[TODO]- Listen to rgbcam data stream (use .listen())
        #[key functions] use .listen(callback))

        # ----------------------------
        # (6) CAM_BACK_LEFT
        # ----------------------------
        #-[TODO]- Get rgbcam information from blueprint (use .get_blueprint_library().find())
        #[key functions] get_blueprint_library().find("sensor.????")
        # rgb camera id: "sensor.camera.rgb"
        rgbcam_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")

        #-[TODO]- Set rgbcam attribute (use .set_attribute())
        #[key functions] set_attribute("attribute", "value"))
        rgbcam_blueprint.set_attribute('image_size_x', str(CAM_W))
        rgbcam_blueprint.set_attribute('image_size_y', str(CAM_H))
        rgbcam_blueprint.set_attribute('fov', '90.0')

        #-[TODO]- Set rgbcam transform
        cam_back_left_transform = carla.Transform(carla.Location(x=0.0, y=-1.0, z=2.0), carla.Rotation(roll=0.0, pitch=0.0, yaw=-135.0))

        #-[TODO]- Spawn rgbcam actor on ego_vehicle (use .spawn_actor())
        #[key functions] spawn_actor(blueprint, transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_back_left = world.spawn_actor(rgbcam_blueprint, cam_back_left_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        cam_sensors.append(cam_back_left)

        def cam_back_left_cb(image):
            nonlocal cam_surfaces
            image.convert(carla.ColorConverter.Raw)
            cam_surfaces[5] = image_to_surface(image)

        #-[TODO]- Listen to rgbcam data stream (use .listen())
        #[key functions] use .listen(callback))

        # ============================================================
        # [TODO] Spawn and attach GNSS
        # ============================================================
        #-[TODO]- Get gnss information from blueprint (use .get_blueprint_library().find())
        #[key functions] get_blueprint_library().find("sensor.????")
        # gnss id: "sensor.other.gnss"
        gnss_blueprint = world.get_blueprint_library().find("sensor.other.gnss")

        #-[TODO]- Set gnss attribute (use .set_attribute())
        #[key functions] set_attribute("attribute", "value"))

        #-[TODO]- Set gnss transform
        gnss_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.0), carla.Rotation(roll=0.0, pitch=0.0, yaw=0.0))

        #-[TODO]- Spawn gnss actor on ego_vehicle (use .spawn_actor())
        #[key functions] spawn_actor(blueprint, transform, attach_to=ego_vehicle)
        gnss_sensor = world.spawn_actor(gnss_blueprint, gnss_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)

        def gnss_cb(data):
            nonlocal last_gnss
            last_gnss = data

        #-[TODO]- Listen to gnss data stream (use .listen())
        #[key functions] use .listen(callback))

        # ============================================================
        # [TODO] Spawn and attach IMU
        # ============================================================
        #-[TODO]- Get imu information from blueprint (use .get_blueprint_library().find())
        #[key functions] get_blueprint_library().find("sensor.????")
        # imu id: "sensor.other.imu"
        imu_blueprint = world.get_blueprint_library().find("sensor.other.imu")

        #-[TODO]- Set imu attribute (use .set_attribute())
        #[key functions] set_attribute("attribute", "value"))

        #-[TODO]- Set imu transform
        imu_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0), carla.Rotation(roll=0.0, pitch=0.0, yaw=0.0))

        #-[TODO]- Spawn imu actor on ego_vehicle (use .spawn_actor())
        #[key functions] spawn_actor(blueprint, transform, attach_to=ego_vehicle)
        imu_sensor = world.spawn_actor(imu_blueprint, imu_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)

        def imu_cb(data):
            nonlocal last_imu_lines
            ax, ay, az = data.accelerometer.x, data.accelerometer.y, data.accelerometer.z
            gx, gy, gz = data.gyroscope.x, data.gyroscope.y, data.gyroscope.z
            last_imu_lines = [
                f"acc=({ax:+.3f}, {ay:+.3f}, {az:+.3f}) m/s^2",
                f"gyro=({gx:+.3f}, {gy:+.3f}, {gz:+.3f}) rad/s",
                f"compass={data.compass:+.3f}",
            ]

        #-[TODO]- Listen to imu data stream (use .listen())
        #[key functions] use .listen(callback))

        # ============================================================
        # Main loop
        # ============================================================
        call_exit = False
        while True:
            world.tick()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True

                if event.type == pygame.VIDEORESIZE:
                    screen = pygame.display.set_mode(event.size, flags)

                if event.type == pygame.KEYDOWN:
                    mods = pygame.key.get_mods()
                    if event.key == K_ESCAPE or (event.key == pygame.K_q and (mods & pygame.KMOD_CTRL)):
                        call_exit = True
                    elif event.key == pygame.K_c:
                        cam_mode_idx = (cam_mode_idx + 1) % len(SPECTATOR_MODES)
                    elif event.key == pygame.K_v:
                        follow_enabled = not follow_enabled
                        if (not follow_enabled) and spectator and map_view_tf:
                            spectator.set_transform(map_view_tf)
                    elif event.key == pygame.K_q:
                        manual_enabled = not manual_enabled
                        reverse_toggle = False
                        steer_cache = 0.0
                        if manual_enabled:
                            ego.set_autopilot(False, tm_port)
                            print("[AUTOPILOT] OFF, MANUAL CONTROL ENABLED")
                        else:
                            ego.set_autopilot(True, tm_port)
                            print("[AUTOPILOT] ON, MANUAL CONTROL DISABLED")
                    elif event.key == pygame.K_r:
                        if manual_enabled:
                            reverse_toggle = not reverse_toggle

            if call_exit:
                break

            if manual_enabled and is_alive(ego):
                keys = pygame.key.get_pressed()
                ctrl, steer_cache = build_vehicle_control(keys, steer_cache, reverse_toggle, MANUAL_CFG)
                ego.apply_control(ctrl)

            if follow_enabled and spectator and is_alive(ego):
                mode = SPECTATOR_MODES[cam_mode_idx]
                spectator.set_transform(spectator_follow_tf(ego.get_transform(), mode, SPEC_CFG))

            now = time.time()
            if now - last_print_t >= float(TERM_PRINT_INTERVAL_SEC):
                last_print_t = now
                if last_gnss is not None:
                    print(f"[GNSS] lat={last_gnss.latitude:.6f}, lon={last_gnss.longitude:.6f}, alt={last_gnss.altitude:.2f}")
                if last_imu_lines is not None:
                    print("[IMU] " + " | ".join(last_imu_lines))

            win_w, win_h = screen.get_size()
            cell_w = max(1, win_w // GRID_COLS)
            cell_h = max(1, win_h // GRID_ROWS)

            screen.fill((0, 0, 0))

            for i in range(6):
                col = i % GRID_COLS
                row = i // GRID_COLS
                x0 = col * cell_w
                y0 = row * cell_h

                pygame.draw.rect(screen, (0, 0, 0), (x0, y0, cell_w, cell_h))

                surf = cam_surfaces[i]
                if surf is not None:
                    blit_fit_keep_aspect(screen, surf, x0, y0, cell_w, cell_h)
                else:
                    pygame.draw.rect(screen, (20, 20, 20), (x0, y0, cell_w, cell_h))
                    msg = font.render("Waiting camera...", True, (220, 220, 220))
                    screen.blit(msg, (x0 + 10, y0 + 10))

                name = CAM_CONFIGS[i][0]
                label_bg = pygame.Surface((220, 24))
                label_bg.set_alpha(160)
                label_bg.fill((0, 0, 0))
                screen.blit(label_bg, (x0 + 8, y0 + 8))
                screen.blit(font_small.render(name, True, (255, 255, 255)), (x0 + 12, y0 + 11))

                pygame.draw.rect(screen, (40, 40, 40), (x0, y0, cell_w, cell_h), 1)

            hud = [
                "B2D Sensors",
            ]
            y = 8
            for line in hud:
                screen.blit(font.render(line, True, (240, 240, 240)), (10, y))
                y += 22

            gnss_frame = base_map_surface.copy()
            if map_cache is not None and is_alive(ego):
                loc = ego.get_transform().location
                ex, ey = world_to_screen(map_cache, loc.x, loc.y)
                pygame.draw.circle(gnss_frame, EGO_DOT_COLOR, (ex, ey), EGO_DOT_RADIUS)

                fwd = ego.get_transform().get_forward_vector()
                hx, hy = world_to_screen(
                    map_cache,
                    loc.x + fwd.x * EGO_HEADING_LEN_M,
                    loc.y + fwd.y * EGO_HEADING_LEN_M
                )
                pygame.draw.line(gnss_frame, EGO_DOT_COLOR, (ex, ey), (hx, hy), 2)

            gy = 8
            gnss_frame.blit(font.render("GNSS(+MAP)", True, (240, 240, 240)), (10, gy))
            gy += 22
            if last_gnss is not None:
                gnss_frame.blit(font.render(f"lat={last_gnss.latitude:.6f}", True, (240, 240, 240)), (10, gy)); gy += 22
                gnss_frame.blit(font.render(f"lon={last_gnss.longitude:.6f}", True, (240, 240, 240)), (10, gy)); gy += 22
                gnss_frame.blit(font.render(f"alt={last_gnss.altitude:.2f} m", True, (240, 240, 240)), (10, gy)); gy += 22

            if gnss_window is not None and not gnss_fallback_inset:
                gnss_window.draw_surface(gnss_frame)
            else:
                inset_w = min(480, max(200, win_w // 5))
                inset_h = inset_w
                inset = pygame.transform.smoothscale(gnss_frame, (inset_w, inset_h))
                ix = win_w - inset_w - 10
                iy = win_h - inset_h - 10
                pygame.draw.rect(screen, (200, 200, 200), (ix - 2, iy - 2, inset_w + 4, inset_h + 4), 2)
                screen.blit(inset, (ix, iy))

            pygame.display.flip()

    finally:
        try:
            if imu_sensor is not None:
                imu_sensor.stop()
                imu_sensor.destroy()
        except Exception:
            pass
        try:
            if gnss_sensor is not None:
                gnss_sensor.stop()
                gnss_sensor.destroy()
        except Exception:
            pass
        try:
            for s in cam_sensors:
                try:
                    s.stop()
                    s.destroy()
                except Exception:
                    pass
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
            if gnss_window is not None:
                gnss_window.close()
        except Exception:
            pass

        try:
            pygame.quit()
        except Exception:
            pass


if __name__ == "__main__":
    main()