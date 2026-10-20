#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_8_manualcontroller.py
# Description: Control vehicle with manual input
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      Feb  20, 2026: Seokhwan Jeong - Created.
#      Feb  21, 2026: Seokhwan Jeong - Added Manual Input
#      Feb  22, 2026: Seokhwan Jeong - Made Empty Template for Student Exercise
#      Jul  03, 2026: Jisang Yun - TODO 주석/변수명 규칙 통일 (노트북 스타일)
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
WINDOW_RES = "500x110"   # 키 입력 전용 창 (차는 CARLA 창에서 확인)
WINDOW_W, WINDOW_H = [int(v) for v in WINDOW_RES.split("x")]

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
# Spectator가 Ego 차량을 따라다니도록 시점 설정하기
# ============================================================
def spectator_follow_tf(ego_transform):
    ##### TODO 1 #####
    # spectator 가 Ego 를 뒤에서 따라보도록 카메라 Transform 구성하기
    # - offset: Ego 로컬 기준 뒤(-8)·위(3) 오프셋 → carla.Location(x=-8.0, y=0.0, z=3.0)
    # - cam_location: offset 을 Ego 로컬→월드 좌표로 변환  사용함수: ego_transform.transform(offset) / 기능: Ego 로컬 좌표를 월드 좌표로 변환
    # - cam_rotation: roll=0, pitch=-12, yaw 는 Ego 와 동일(ego_transform.rotation.yaw)
    #################

    offset = carla.Location(x=..., y=..., z=...)
    cam_location = ego_transform.transform(...)
    cam_rotation = carla.Rotation(roll=..., pitch=..., yaw=...)

    #################

    return carla.Transform(cam_location, cam_rotation)

# ============================================================
# 차량 제어하기
# ============================================================
def build_control(keys, reverse_toggle):
    # 값 초기화
    throttle = 0.0
    brake = 0.0
    steer = 0.0

    ##### TODO 2 #####
    # 키 입력에 따라 throttle / brake / steer 값 채우기
    # - W: 가속(throttle), S: 제동(brake), A: 좌회전, D: 우회전
    # - steer 범위 -1.0~+1.0 (왼쪽은 -, 오른쪽은 +)
    #################

    if keys[pygame.K_w]:
        throttle = ...
    else:
        throttle = ...

    if keys[pygame.K_s]:
        brake = ...
    else:
        brake = ...

    if keys[pygame.K_a] and not keys[pygame.K_d]:
        steer = ...
    elif keys[pygame.K_d] and not keys[pygame.K_a]:
        steer = ...

    #################

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
    tm_port = traffic_manager.get_port()   # tm_port: Traffic Manager 포트 번호

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

    # 상태 변수
    reverse_toggle = False
    call_exit = False

    # ============================================================
    # Ego 차량 Spawn하기
    # ============================================================
    # Ego 차량 blueprint 가져오기 + 속성 설정
    ego_blueprint = world.get_blueprint_library().find('vehicle.nissan.patrol_2021')
    ego_blueprint.set_attribute('role_name', 'ego')
    ego_blueprint.set_attribute('color', "255,0,0")

    # 맵의 스폰 지점 가져오기
    spawn_points = world.get_map().get_spawn_points()

    try:
        # 스폰 지점 선택 후 Ego 차량 스폰
        ego_transform = spawn_points[0]

        ego_vehicle = world.spawn_actor(ego_blueprint, ego_transform)
        # 수동 제어를 위해 autopilot 끄기
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
                ##### TODO 3 #####
                # apply_control() 로 control_input 을 Ego 차량에 적용하기
                # 사용함수: ego_vehicle.apply_control(control_input) / 기능: 차량에 제어 명령 적용
                #################

                ...

                #################

            if is_alive(ego_vehicle) and spectator is not None:
                spectator.set_transform(spectator_follow_tf(ego_vehicle.get_transform()))

            world.tick()
            clock.tick_busy_loop(target_fps)
            hud_lines = [   # 영어로 표시: pygame 기본 글꼴(Arial)은 한글이 깨짐
                "W: accel   S: brake   A / D: steer",
                "R: reverse toggle   ESC: quit",
                "Keys not working? Click this window.",
            ]

            y = 20
            for line in hud_lines:
                text_surface = font.render(line, True, (240, 240, 240))
                screen.blit(text_surface, (20, y))
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
