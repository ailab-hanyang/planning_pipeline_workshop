#!/usr/bin/env python3
#======================================================================#
# Module:      P_Basic_7_manualcontrol.py
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

    # ============================================================
    # Spectator Transform 설정
    # ============================================================
    spectator = world.get_spectator()

    ##### TODO 1 #####
    # spectator 를 Ego 차량 위 50m 에서 수직으로 내려다보도록 위치/자세 정하기
    # - 위치: x, y 는 Ego 와 동일, z 는 Ego + 50
    # - 자세: roll=0, pitch=-90(바로 아래), yaw=0
    # 사용함수: carla.Location(x=, y=, z=) / carla.Rotation(pitch=, yaw=, roll=) / 기능: 위치·자세(회전) 값 생성
    # 주의: Rotation 위치 인자 순서 = (pitch, yaw, roll) -> 항상 이름=값 으로 작성
    #################

    top_location = carla.Location(
        x=...,
        y=...,
        z=...
    )
    top_rotation = carla.Rotation(
        roll=...,
        pitch=...,
        yaw=...
    )

    #################

    ##### TODO 2 #####
    # top_location 과 top_rotation 으로 Transform 을 만들어 spectator 에 설정하기
    # 사용함수: spectator.set_transform(carla.Transform(location, rotation)) / 기능: 관전 카메라를 지정 위치/자세로 이동
    #################

    ...

    #################

    # ============================================================
    # Manual Control 입력받아서 차량 제어하기
    # ============================================================
    ego_vehicle.set_autopilot(False)

    ##### TODO 3 #####
    # throttle, steer, brake, reverse 값을 넣어 VehicleControl 만들기 (steer +는 우회전, -는 좌회전)
    # 사용함수: carla.VehicleControl(throttle=, steer=, brake=, reverse=) / 기능: 차량 제어 명령 생성 (throttle/brake 0~1, steer -1~1)
    # 주의: 위치 인자 순서 = (throttle, steer, brake, hand_brake, reverse) -> 4번째는 hand_brake, 이름=값 으로 작성
    #################

    control_input = carla.VehicleControl(
        throttle=...,
        steer=...,
        brake=...,
        reverse=bool(...)
    )

    #################

    print("[INFO] Manual control start")

    while True:
        ##### TODO 4 #####
        # apply_control() 로 control_input 을 Ego 차량에 적용하기
        # 사용함수: ego_vehicle.apply_control(control_input) / 기능: 차량에 제어 명령 적용
        #################

        ...

        #################
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
