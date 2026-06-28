#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_8_manualcontroller.py
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

try:
    import pygame
    from pygame.locals import K_ESCAPE
except ImportError:
    raise RuntimeError("cannot import pygame, make sure pygame is installed")

# ============================================================
# 파라미터
# ============================================================
WINDOW_RES = "500x40"
WINDOW_W, WINDOW_H = [int(x) for x in WINDOW_RES.split("x")]

# ============================================================
# 함수 정의
# ============================================================
def is_alive(actor):
    try:
        return actor is not None and actor.is_alive
    except Exception:
        return False

def destroy_actor_safe(actor):
    if actor is None:
        return
    try:
        actor.destroy()
    except Exception:
        pass

# ============================================================
# -[TODO]- Spectator가 Ego 차량을 따라다니도록 시점 설정하기
# ============================================================
def spectator_follow_tf(ego_transform):
    #-[TODO]-Add offset to the ego vehicle location to set the spectator location (x: -8.0, y: 0.0, z: 3.0)
    offset = carla.Location(x=-8.0, y=0.0, z=3.0)
    #-[TODO]-Set Camera location on ego frame
    #[key functions] tranform(offset)
    cam_location = ego_transform.transform(offset)
    #-[TODO]-Set the spectator rotation to look at the ego vehicle (roll: 0.0, pitch: -12.0, yaw: same as ego)
    cam_rotation = carla.Rotation(roll=0.0, pitch=-12.0, yaw=ego_transform.rotation.yaw)

    return carla.Transform(cam_location, cam_rotation)

# ============================================================
# -[TODO]- 차량 제어하기 
# ============================================================
def build_control(keys, reverse_toggle):
    # value reset
    throttle = 0.0
    brake = 0.0
    steer = 0.0

    #-[TODO]-Accelerate when W key is pressed
    if keys[pygame.K_w]:
        throttle = 0.7
    else:
        throttle = 0.0

    #-[TODO]-Brake when S key is pressed
    if keys[pygame.K_s]:
        brake = 1.0
    else:
        brake = 0.0
    
    #-[TODO]-Steer left when A key is pressed, steer right when D key is pressed (steer value range: -1.0 to +1.0, where negative is left and positive is right)]
    #-Left : (-), Right : (+)
    if keys[pygame.K_a] and not keys[pygame.K_d]:
        steer = -1.0
    elif keys[pygame.K_d] and not keys[pygame.K_a]:
        steer = 1.0

    control_cmd = carla.VehicleControl(
        throttle=throttle,
        steer=steer,
        brake=brake,
        reverse=bool(reverse_toggle)
    )

    return control_cmd

def main():
    pygame.init()

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
    # Pygame 화면 생성
    # ============================================================
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("CARLA | Simple EGO Manual | WASD + R reverse | ESC quit")
    font = pygame.font.SysFont("Arial", 20)

    # tick 주기 고정
    clock = pygame.time.Clock()
    target_fps = int(round(1.0 / max(0.05, 1e-6)))

    ego_vehicle = None
    spectator = world.get_spectator()

    # state
    reverse_toggle = False
    call_exit = False

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
        #Set the ego vehicle to autopilot
        ego_vehicle.set_autopilot(False)

        world.tick()

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True

                elif event.type == pygame.KEYDOWN:
                    mods = pygame.key.get_mods()

                    if event.key == K_ESCAPE or (event.key == pygame.K_q and (mods & pygame.KMOD_CTRL)):
                        call_exit = True

                    elif event.key == pygame.K_r:
                        reverse_toggle = not reverse_toggle

            if call_exit:
                break

            keys = pygame.key.get_pressed()
            if is_alive(ego_vehicle):
                control_input = build_control(keys, reverse_toggle)
                #-[TODO]-use apply_control() to apply the control to the ego vehicle
                #[key functions] apply_control(input)
                ego_vehicle.apply_control(control_input)

            if is_alive(ego_vehicle) and spectator is not None:
                spectator.set_transform(spectator_follow_tf(ego_vehicle.get_transform()))

            world.tick()
            clock.tick_busy_loop(target_fps)
            hud_lines = [
                "P_Basic_8_manualcontroller.py",
            ]

            y = 20
            for line in hud_lines:
                surf = font.render(line, True, (240, 240, 240))
                screen.blit(surf, (20, y))
                y += 26

            pygame.display.flip()

    finally:
        try:
            destroy_actor_safe(ego_vehicle)
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

        try:
            pygame.quit()
        except Exception:
            pass


if __name__ == "__main__":
    main()