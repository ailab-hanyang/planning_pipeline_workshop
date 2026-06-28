#!/usr/bin/env python3
#======================================================================#
# Module:      P_Planning_1_force_lane_change.py
# Description: Practice forcing lane changes with Traffic Manager force_lane_change()
#
# Authors: Jisang Yun (jisangyun@hanyang.ac.kr)
#
# Revision History
#      June 28, 2026: Jisang Yun - Created.
#======================================================================#
import math
import time
import carla

# ============================================================
# 설정값 (여기만 바꾸면 동작이 달라집니다)
# ============================================================
MAP_NAME       = 'Town04'   # 고속도로(직선 다차선)라 차선변경 관찰이 쉬움
FIXED_DELTA_S  = 0.05
TARGET_KMH     = 24.0       # 목표 주행 속도 (cut-in 예제처럼 너무 빠르지 않게)

# 한 번만 차선변경을 실습한다. 'left' 또는 'right' 로 바꿔서 실행하면 된다.
LANE_CHANGE_DIRECTION = 'left'
WARMUP_SECONDS  = 4.0       # 주행 시작 후 첫 차선변경까지 대기 (s)
SETTLE_SECONDS  = 2.0       # 새 차선에 들어간 뒤 안정화 확인 시간 (s)
ATTEMPT_TIMEOUT = 8.0       # 한 방향이 이 시간 내 안 바뀌면 포기하고 다음으로 (s)
RUN_SECONDS     = 30.0      # 전체 실행 시간 (s)

# ============================================================
# Util
# ============================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def get_speed_kmh(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x**2 + v.y**2 + v.z**2) * 3.6

def set_tm_speed(traffic_manager, vehicle, target_kmh):
    # TM 은 '제한속도 대비 퍼센트 차이'로 속도를 지정한다.
    limit = max(1.0, float(vehicle.get_speed_limit()))
    pct   = ((limit - float(target_kmh)) / limit) * 100.0
    traffic_manager.vehicle_percentage_speed_difference(vehicle, clamp(pct, -80.0, 95.0))

def set_initial_velocity(vehicle, kmh):
    # 정지 출발이 아니라 처음부터 달리는 상태로 (차선변경이 잘 먹히도록)
    fwd = vehicle.get_transform().get_forward_vector()
    mps = kmh / 3.6
    vehicle.set_target_velocity(carla.Vector3D(fwd.x * mps, fwd.y * mps, 0.0))

def get_lane_id(world, vehicle):
    wp = world.get_map().get_waypoint(
        vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    return wp.lane_id if wp else None

def adjacent_driving_lane(wp, to_right):
    # 요청 방향에 '같은 진행방향의 주행차선'이 있으면 그 waypoint, 없으면 None
    if wp is None:
        return None
    a = wp.get_right_lane() if to_right else wp.get_left_lane()
    if a is None or a.lane_type != carla.LaneType.Driving:
        return None
    if (a.lane_id < 0) != (wp.lane_id < 0):   # 부호가 같아야 같은 방향
        return None
    return a

def destroy_all_vehicles(world):
    vehicles = world.get_actors().filter('vehicle.*')
    for v in vehicles:
        v.destroy()
    if len(vehicles) > 0:
        print(f'[INFO] 기존 차량 정리: {len(vehicles)}대')

def find_middle_lane_spawn(world):
    # 좌/우 양쪽에 주행차선이 있는 (가운데) spawn point 를 찾는다
    amap = world.get_map()
    for sp in amap.get_spawn_points():
        wp = amap.get_waypoint(sp.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if adjacent_driving_lane(wp, True) is not None and adjacent_driving_lane(wp, False) is not None:
            return sp
    return None

def chase_camera_tf(ego_transform):
    # 차 뒤쪽 위에서 비스듬히 따라보는 시점
    offset = carla.Location(x=-8.0, y=0.0, z=4.0)
    loc = ego_transform.transform(offset)
    rot = carla.Rotation(roll=0.0, pitch=-15.0, yaw=ego_transform.rotation.yaw)
    return carla.Transform(loc, rot)

def setup_ego_for_lane_change(traffic_manager, tm_port, ego_vehicle, target_kmh):
    # TM 설정은 한 곳에 모아두면 실험 조건을 바꾸기 쉽다.
    set_initial_velocity(ego_vehicle, target_kmh)
    ego_vehicle.set_autopilot(True, tm_port)
    traffic_manager.auto_lane_change(ego_vehicle, True)
    traffic_manager.random_left_lanechange_percentage(ego_vehicle, 0.0)
    traffic_manager.random_right_lanechange_percentage(ego_vehicle, 0.0)
    traffic_manager.keep_slow_lane_rule_percentage(ego_vehicle, 0.0)
    traffic_manager.distance_to_leading_vehicle(ego_vehicle, 5.0)
    traffic_manager.ignore_lights_percentage(ego_vehicle, 100.0)
    traffic_manager.ignore_signs_percentage(ego_vehicle, 100.0)
    set_tm_speed(traffic_manager, ego_vehicle, target_kmh)

def lane_diag_line(world, vehicle):
    wp = world.get_map().get_waypoint(
        vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return 'lane=NONE'
    left_ok = adjacent_driving_lane(wp, False) is not None
    right_ok = adjacent_driving_lane(wp, True) is not None
    return (
        f'speed={get_speed_kmh(vehicle):.1f}km/h  '
        f'lane_id={wp.lane_id}  lane_change={wp.lane_change}  '
        f'left={left_ok} right={right_ok}'
    )

# ============================================================
# Main
# ============================================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    world  = client.load_world(MAP_NAME)
    original_settings = world.get_settings()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_S
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    tm_port = traffic_manager.get_port()

    actor_list = []

    try:
        # ============================================================
        # 기존 차량 정리 (왼/오 차선을 막지 않게)
        # ============================================================
        destroy_all_vehicles(world)
        world.tick()

        # ============================================================
        # 가운데 차선 spawn 선택 + 차량 Spawn
        # ============================================================
        spawn_tf = find_middle_lane_spawn(world)
        if spawn_tf is None:
            raise RuntimeError('좌/우 차선이 모두 있는 가운데 차선 spawn 을 찾지 못함')

        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name', 'ego')
        ego_bp.set_attribute('color', '255,0,0')
        ego_vehicle = world.try_spawn_actor(ego_bp, spawn_tf)
        if ego_vehicle is None:
            raise RuntimeError('ego spawn 실패')
        actor_list.append(ego_vehicle)
        world.tick()

        # ============================================================
        # Autopilot + TM 설정
        # ============================================================
        # 주의: force_lane_change() 는 auto_lane_change 가 True 여야 동작한다.
        setup_ego_for_lane_change(traffic_manager, tm_port, ego_vehicle, TARGET_KMH)

        # force_lane_change 호출 (재요청 시에도 같은 함수 사용)
        def request_lane_change(to_right):
            #-[TODO]- Force the ego vehicle to change lane in the given direction (to_right: True=right)
            #[key functions] traffic_manager.force_lane_change(vehicle, to_right: bool)
            traffic_manager.force_lane_change(ego_vehicle, to_right)

        spectator    = world.get_spectator()
        warmup_tick  = int(WARMUP_SECONDS / FIXED_DELTA_S)
        settle_tick  = int(SETTLE_SECONDS / FIXED_DELTA_S)
        timeout_tick = int(ATTEMPT_TIMEOUT / FIXED_DELTA_S)
        total_tick   = int(RUN_SECONDS / FIXED_DELTA_S)

        print(f'[INFO] 가운데 차선(lane_id={get_lane_id(world, ego_vehicle)})에서 시작.')
        print(f'[INFO] {WARMUP_SECONDS:.0f}초 후 차선변경 실습: {LANE_CHANGE_DIRECTION}. Ctrl+C 종료')

        # ============================================================
        # 단일 차선변경 상태머신
        # ============================================================
        state        = 'warmup'   # warmup → command → wait → hold → done
        to_right     = False
        issue_tick   = 0
        attempt_tick = 0
        hold_tick    = 0
        hold_lane_id = None
        lane_at_issue = None

        for tick in range(total_tick):
            world.tick()
            time.sleep(FIXED_DELTA_S)
            spectator.set_transform(chase_camera_tf(ego_vehicle.get_transform()))

            if state == 'warmup':
                if tick >= warmup_tick:
                    state = 'command'

            if state == 'command':
                direction = LANE_CHANGE_DIRECTION
                to_right  = (direction == 'right')
                lane_at_issue = get_lane_id(world, ego_vehicle)
                attempt_tick = tick
                # 진단: 명령 직전 상태
                print(f'[DIAG] {lane_diag_line(world, ego_vehicle)}')
                request_lane_change(to_right)
                issue_tick = tick
                state = 'wait'
                print(f'[CMD] force_lane_change({direction.upper()}) 실행')

            elif state == 'wait':
                lane_now = get_lane_id(world, ego_vehicle)
                if lane_now is not None and lane_now != lane_at_issue:
                    hold_lane_id = lane_now
                    hold_tick = 0
                    state = 'hold'
                    print(f'[HOLD] 차선 진입 감지: {lane_at_issue} → {lane_now} ({SETTLE_SECONDS:.0f}초 안정화 확인)')
                elif (tick - issue_tick) >= 20 and (tick - issue_tick) % 20 == 0:
                    # 한 번에 안 먹힐 수 있어 20틱마다 재요청
                    request_lane_change(to_right)
                    print(f'[CMD] force_lane_change({LANE_CHANGE_DIRECTION.upper()}) 재요청')
                elif (tick - attempt_tick) >= timeout_tick:
                    # 제한 시간 초과 시 경고만 출력 (spawn 위치/속도 조건 점검 필요)
                    print(f'[WARN] 차선 변경 대기 중 {ATTEMPT_TIMEOUT:.0f}초 초과')
                    attempt_tick = tick

            elif state == 'hold':
                lane_now = get_lane_id(world, ego_vehicle)
                if lane_now == hold_lane_id:
                    hold_tick += 1
                    if hold_tick >= settle_tick:
                        print(f'[DONE] 차선 변경 완료: {lane_at_issue} → {hold_lane_id}')
                        state = 'done'
                elif lane_now is not None:
                    # 차선이 잠깐 흔들리면 다시 대기 상태로 돌아가서 안정화를 다시 본다.
                    lane_at_issue = lane_now
                    state = 'wait'

            elif state == 'done':
                print('[INFO] 차선변경 실습 완료. 계속 주행합니다.')
                break

    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        client.apply_batch([carla.command.DestroyActor(a.id) for a in actor_list])
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print('[DONE] 정리 완료')


if __name__ == '__main__':
    main()
