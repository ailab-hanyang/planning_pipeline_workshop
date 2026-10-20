#!/usr/bin/env python3
#======================================================================#
# Module:      P_TrafficManager_6_force_lane_change.py
# Description: Force a lane change with Traffic Manager force_lane_change()
#              (옆 차선 확인 + lane_id 부호 규칙 -> Planning_3 끼어들기 시나리오에서 사용)
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      June 28, 2026: Jisang Yun - Created. (P_Planning_1_force_lane_change)
#      Oct  19, 2026: Jisang Yun - Traffic Manager 묶음으로 이동
#======================================================================#

import time

import carla

# ============================================================
# Config
# ============================================================
MAP_NAME = 'Town04'
FIXED_DELTA_S = 0.05
LANE_CHANGE_DIRECTION = 'left'  # 'left' or 'right'
START_DELAY_S = 3.0
RUN_SECONDS = 10.0


# ============================================================
# Util
# ============================================================
def has_adjacent_driving_lane(world, spawn_transform, to_right):
    ##### TODO 1 #####
    # 스폰 위치를 도로 위 가장 가까운 waypoint 로 투영하기 (주행 차선만 대상)
    # 사용함수: world.get_map().get_waypoint(location, project_to_road, lane_type) / 기능: 위치를 가장 가까운 차선 waypoint 로 투영 (lane_type 으로 주행 차선만 남김)
    #################

    waypoint = world.get_map().get_waypoint(spawn_transform.location, project_to_road=True, lane_type=...)

    #################
    if waypoint is None:
        return False

    ##### TODO 2 #####
    # 이동하려는 방향의 옆 차선 waypoint 가져오기 (오른쪽은 예시로 채워져 있음)
    # 사용함수: waypoint.get_right_lane() / waypoint.get_left_lane() / 기능: 바로 오른쪽/왼쪽 차선의 waypoint 반환
    #################

    adjacent_waypoint = waypoint.get_right_lane() if to_right else ...

    #################
    if adjacent_waypoint is None or adjacent_waypoint.lane_type != carla.LaneType.Driving:
        return False

    ##### TODO 3 #####
    # 같은 진행 방향인지 반환하기: 두 lane_id 부호가 같으면 True
    # (lane_id 는 진행 방향에 따라 +/- 로 갈림)
    #################

    return (adjacent_waypoint.lane_id < 0) == ...

    #################


def has_both_adjacent_driving_lanes(world, spawn_transform):
    # 왼쪽/오른쪽 인접 주행 차선이 '둘 다' 있을 때만 True
    return has_adjacent_driving_lane(world, spawn_transform, True) and \
        has_adjacent_driving_lane(world, spawn_transform, False)


def find_spawn_point(world, to_right):
    spawn_points = world.get_map().get_spawn_points()

    # 우선 왼쪽/오른쪽 인접 차선이 둘 다 있는 가운데 차선을 선택
    for spawn_transform in spawn_points:
        if has_both_adjacent_driving_lanes(world, spawn_transform):
            return spawn_transform

    # 차선책: 최소한 변경하려는 방향의 인접 차선이 있는 곳
    for spawn_transform in spawn_points:
        if has_adjacent_driving_lane(world, spawn_transform, to_right):
            return spawn_transform

    return None


def follow_camera_tf(ego_transform):
    # 차량 뒤쪽 위에서 따라보는 간단한 spectator 시점
    offset = carla.Location(x=-8.0, y=0.0, z=4.0)
    cam_location = ego_transform.transform(offset)
    cam_rotation = carla.Rotation(roll=0.0, pitch=-15.0, yaw=ego_transform.rotation.yaw)
    return carla.Transform(cam_location, cam_rotation)


def print_vehicle_location(vehicle):
    location = vehicle.get_location()
    print(f'[POS] x={location.x:.1f}, y={location.y:.1f}, z={location.z:.1f}')


# ============================================================
# Main
# ============================================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    world = client.load_world(MAP_NAME)
    original_settings = world.get_settings()

    # ============================================================
    # Synchronous mode 설정
    # ============================================================
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_S
    world.apply_settings(settings)

    # ============================================================
    # Traffic Manager 연결
    # ============================================================
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    tm_port = traffic_manager.get_port()   # tm_port: Traffic Manager 포트 번호

    ego_vehicle = None

    try:
        # ============================================================
        # 기존 차량 정리
        # ============================================================
        for vehicle in world.get_actors().filter('vehicle.*'):
            vehicle.destroy()
        world.tick()

        # ============================================================
        # Spawn point / Ego 차량 준비
        # ============================================================
        to_right = (LANE_CHANGE_DIRECTION == 'right')
        spawn_transform = find_spawn_point(world, to_right)
        if spawn_transform is None:
            raise RuntimeError('차선 변경 가능한 spawn point 를 찾지 못함')

        ego_blueprint = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_blueprint.set_attribute('role_name', 'ego')

        ego_vehicle = world.try_spawn_actor(ego_blueprint, spawn_transform)
        if ego_vehicle is None:
            raise RuntimeError('ego spawn 실패')

        ##### TODO 4 #####
        # Ego 를 Traffic Manager 에 넘겨 스스로 주행하게 하기
        # 사용함수: vehicle.set_autopilot(enabled, tm_port) / 기능: 지정한 TM 포트로 해당 차량의 Traffic Manager autopilot 활성화
        #################

        ego_vehicle.set_autopilot(True, ...)

        #################

        ##### TODO 5 #####
        # 차선 변경 허용하기
        # 주의: 허용 안 함 -> 아래 force_lane_change 무시됨
        # 사용함수: traffic_manager.auto_lane_change(vehicle, enabled) / 기능: 해당 차량의 차선 변경을 TM 이 수행하도록 허용/차단
        #################

        traffic_manager.auto_lane_change(ego_vehicle, ...)

        #################

        spectator = world.get_spectator()
        world.tick()   # 스폰 직후엔 위치가 (0,0,0) 으로 읽히므로 한 번 진행
        start_location = ego_vehicle.get_location()

        print(f'[START] x={start_location.x:.1f}, y={start_location.y:.1f}, z={start_location.z:.1f}')
        print(f'[INFO] {LANE_CHANGE_DIRECTION.upper()} lane change test start')

        # ============================================================
        # Lane change trigger
        # ============================================================
        start_delay_tick = int(START_DELAY_S / FIXED_DELTA_S)
        total_tick = int(RUN_SECONDS / FIXED_DELTA_S)

        for tick in range(total_tick):
            if tick == start_delay_tick:
                ##### TODO 6 #####
                # 지연 시간이 지나면 선택한 방향으로 즉시 차선 변경 명령하기
                # 사용함수: traffic_manager.force_lane_change(vehicle, to_right) / 기능: 즉시 1회 차선 변경 강제 (True=오른쪽, False=왼쪽)
                #################

                traffic_manager.force_lane_change(ego_vehicle, ...)

                #################
                print(f'[CMD] force_lane_change({LANE_CHANGE_DIRECTION.upper()})')

            world.tick()
            spectator.set_transform(follow_camera_tf(ego_vehicle.get_transform()))

            # 1초에 한 번 Ego 위치를 출력해 맵 상 어디쯤인지 확인
            if tick % int(1.0 / FIXED_DELTA_S) == 0:
                print_vehicle_location(ego_vehicle)

            time.sleep(FIXED_DELTA_S)

        print('[DONE] test finished')

    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()

        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print('[DONE] 정리 완료')


if __name__ == '__main__':
    main()
