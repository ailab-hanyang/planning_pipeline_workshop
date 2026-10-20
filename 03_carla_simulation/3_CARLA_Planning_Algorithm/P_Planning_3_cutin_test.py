#!/usr/bin/env python3
#======================================================================#
# Module:      P_Planning_3_cutin_test.py
# Description: 시나리오 테스트: 끼어들기(cut-in) 상황에서 Planning_2 판단기를 시험·평가
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      June 28, 2026: Jisang Yun - Created. (P_Planning_3_scenario_cutin)
#      Oct  19, 2026: Jisang Yun - Planning_2 판단 함수 재사용, 끼어들기 거리 실험·평가 정리
#======================================================================#
# 시나리오 테스트 = 같은 판단기를 조건만 바꿔 반복 실행 -> 어디까지 안전한지 숫자로 확인
#   먼저 P_Planning_2_ttc_braking.py 의 TODO 1~3 완성 (TTC·필요 감속도·판단 함수를 그대로 가져와 사용)
#
# [실험] 끼어드는 거리를 줄여 가며 결과 표 채우기
#   python3 P_Planning_3_cutin_test.py 50     (기본값)
#   python3 P_Planning_3_cutin_test.py 30
#   python3 P_Planning_3_cutin_test.py 20
#   -> 실행 끝에 나오는 [TABLE] 줄 비교 (그 아래 [INFO]·[RESULT]·[DONE] 줄이 더 나옴)
#======================================================================#
import csv
import math
import sys
import time
from pathlib import Path
import carla

# Planning_2 에서 완성한 판단 함수 재사용 (같은 폴더)
from P_Planning_2_ttc_braking import (compute_ttc, required_decel, decide, lane_keep_steer,
                                      BRAKE_LEVEL, TTC_RESUME, SAFE_GAP_M)

# ============================================================
# 설정값
# ============================================================
# Town04 고속도로 spawn point: 가운데 차선(양옆 차선 있음), 450 m 앞까지 직선, 진행 방향 yaw -90°
# (yaw ±180° 근처 도로는 TM 조향이 각도 경계에서 좌우로 떨림 -> 피함)
EGO_SPAWN_INDEX = 124
ADV_FORWARD_M         = 200.0   # Ego 전방 adversary(끼어드는 차) 스폰 거리
EGO_TARGET_KMH        = 50.0
ADV_TARGET_KMH        = 20.0
ADV_CUTIN_KMH         = 18.0   # cut-in 중 adversary 감속 속도
TRIGGER_DISTANCE_M    = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0   # 이 거리 안에 들어오면 cut-in 발동 (실행 인자로 변경)
MAX_TICKS             = 5000
POST_CUTIN_SECONDS    = 8.0    # cut-in 완료 후 추가 추종 시간
FIXED_DELTA_S         = 0.05
LOG_PATH              = Path('output/cutin_log.csv')

VEH_LEN_M     = 4.5   # 차량 길이 보정 (중심 거리 -> 차간 간격)
EGO_LANE_HALF = 2.5   # (m) |횡거리| < 이 값 -> 내 차선 침범으로 판정 (옆 차선 중앙 3.5 m 는 제외)
                      #     참고: 차체가 차선에 닿는 거리 = 차선 반폭 1.75 + 차 반폭 약 0.9 ≈ 2.65

# ============================================================
# Util
# ============================================================
def clamp(value, low, high):
    return max(low, min(high, value))

def get_speed_kmh(vehicle):
    velocity = vehicle.get_velocity()
    speed_mps = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    return speed_mps, speed_mps * 3.6

def set_tm_speed(traffic_manager, vehicle, target_kmh):
    speed_limit = max(1.0, float(vehicle.get_speed_limit()))
    # speed_diff_pct: 제한속도 대비 몇 % 느리게(+)/빠르게(-) 달릴지
    speed_diff_pct = ((speed_limit - float(target_kmh)) / speed_limit) * 100.0
    traffic_manager.vehicle_percentage_speed_difference(vehicle, clamp(speed_diff_pct, -80.0, 95.0))

def get_lane_info(world, vehicle):
    waypoint = world.get_map().get_waypoint(
        vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    return (waypoint.road_id, waypoint.lane_id) if waypoint else (None, None)

def draw_label(world, actor, text, color):
    # life_time 기본값(-1) = 한 프레임만 표시 (1 tick 로 주면 이전 프레임 글씨와 겹쳐 2개로 보임)
    world.debug.draw_string(
        actor.get_location() + carla.Location(z=2.5),
        text, draw_shadow=False, color=color, persistent_lines=False)

# ============================================================
# cut-in 방향 추론
# ============================================================
def infer_cutin_direction_is_right(world, adv_vehicle, ego_vehicle):
    carla_map    = world.get_map()
    adv_waypoint = carla_map.get_waypoint(adv_vehicle.get_location(),
                                          project_to_road=True, lane_type=carla.LaneType.Driving)
    ego_waypoint = carla_map.get_waypoint(ego_vehicle.get_location(),
                                          project_to_road=True, lane_type=carla.LaneType.Driving)
    if adv_waypoint is None or ego_waypoint is None:
        return True
    right_waypoint = adv_waypoint.get_right_lane()
    left_waypoint  = adv_waypoint.get_left_lane()
    if right_waypoint is not None and right_waypoint.lane_id == ego_waypoint.lane_id:
        return True
    if left_waypoint is not None and left_waypoint.lane_id == ego_waypoint.lane_id:
        return False
    return abs(ego_waypoint.lane_id) > abs(adv_waypoint.lane_id)

# ============================================================
# Spectator
# ============================================================
def chase_camera_tf(ego_transform):
    # 차 뒤·위에서 따라보는 고정 카메라
    offset = carla.Location(x=-8.0, y=0.0, z=4.0)
    cam_location = ego_transform.transform(offset)
    cam_rotation = carla.Rotation(roll=0.0, pitch=-15.0, yaw=ego_transform.rotation.yaw)
    return carla.Transform(cam_location, cam_rotation)

# ============================================================
# Main
# ============================================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world  = client.load_world('Town04')
    original_settings = world.get_settings()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_S
    world.apply_settings(settings)
    delta_seconds = settings.fixed_delta_seconds   # delta_seconds: 한 tick 의 시간 (s)

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    tm_port = traffic_manager.get_port()   # tm_port: Traffic Manager 포트 번호

    actor_list    = []
    log_rows      = []
    collision_info = {'detected': False, 'other': None}

    cut_in_triggered = False
    cut_in_completed = False
    target_lane_id   = None
    force_dir_right  = True
    same_lane_hold   = 0
    prev_ego_speed_mps = None
    react_gap_m, react_ttc = None, None   # 첫 제동 반응 시점의 간격·TTC (평가용)
    min_gap_m        = float('inf')
    emergency_count  = 0
    cooldown_left    = int(POST_CUTIN_SECONDS / FIXED_DELTA_S)
    ego_autopilot    = True            # Ego 는 평소 autopilot, TTC 위험 시 직접 제동
    ego_braked       = False           # 한 번이라도 TTC 제동했는지 (평가용)
    result, reason   = 'RUNNING', ''

    try:
        # 기존 차량 정리
        for vehicle in world.get_actors().filter('vehicle.*'):
            vehicle.destroy()
        world.tick()

        # ============================================================
        # Ego Spawn
        # ============================================================
        ego_blueprint = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
        ego_blueprint.set_attribute('role_name', 'ego')
        ego_blueprint.set_attribute('color', '255,0,0')
        ego_spawn = world.get_map().get_spawn_points()[EGO_SPAWN_INDEX]
        ego_vehicle = world.try_spawn_actor(ego_blueprint, carla.Transform(
            ego_spawn.location + carla.Location(z=0.3), ego_spawn.rotation))
        if ego_vehicle is None:
            raise RuntimeError('ego spawn 실패')
        actor_list.append(ego_vehicle)
        print(f'[SPAWN] Ego at spawn[{EGO_SPAWN_INDEX}] ({ego_spawn.location.x:.1f}, {ego_spawn.location.y:.1f})')
        world.tick()

        # ============================================================
        # Adversary: Ego 전방 인접 차선
        # ============================================================
        ego_waypoint = world.get_map().get_waypoint(
            ego_vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)

        ##### TODO 1 #####
        # 차선을 따라 Ego 전방 ADV_FORWARD_M 만큼 앞의 지점 구하기
        # 사용함수: ego_waypoint.next(distance) / 기능: 차선을 따라 distance(m) 앞의 waypoint 목록 반환 (list[Waypoint])
        #################

        ahead_waypoints = ego_waypoint.next(...)

        #################
        if not ahead_waypoints:
            raise RuntimeError('전방 waypoint 없음')

        ##### TODO 2 #####
        # 인접 차선 waypoint 가져오기: 오른쪽을 먼저 시도하고 없으면 왼쪽으로 대체 (오른쪽은 예시로 채워져 있음)
        # 사용함수: waypoint.get_right_lane() / waypoint.get_left_lane() / 기능: 바로 오른쪽/왼쪽 차선의 waypoint 반환 (없으면 None)
        #################

        adjacent_waypoint = ahead_waypoints[0].get_right_lane() or ...

        #################
        if adjacent_waypoint is None or adjacent_waypoint.lane_type != carla.LaneType.Driving:
            raise RuntimeError('인접 주행 차선 없음 (갓길 등은 제외)')

        adv_blueprint = world.get_blueprint_library().filter('vehicle.audi.tt')[0]
        adv_vehicle = world.try_spawn_actor(adv_blueprint, carla.Transform(
            carla.Location(adjacent_waypoint.transform.location.x,
                           adjacent_waypoint.transform.location.y,
                           adjacent_waypoint.transform.location.z + 0.3),
            adjacent_waypoint.transform.rotation))
        if adv_vehicle is None:
            raise RuntimeError('adversary spawn 실패 — ADV_FORWARD_M 값을 바꿔보세요')
        actor_list.append(adv_vehicle)
        print(f'[SPAWN] Adversary {ADV_FORWARD_M:.0f}m 전방 인접 차선')
        world.tick()

        # ============================================================
        # 충돌 센서
        # ============================================================
        collision_blueprint = world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor = world.spawn_actor(collision_blueprint, carla.Transform(), attach_to=ego_vehicle)
        collision_sensor.listen(lambda event: collision_info.update(detected=True, other=event.other_actor.type_id))
        actor_list.append(collision_sensor)

        # ============================================================
        # 초기 속도 + Autopilot + TM 설정
        # ============================================================
        ego_vehicle.set_autopilot(True, tm_port)
        adv_vehicle.set_autopilot(True, tm_port)
        traffic_manager.auto_lane_change(ego_vehicle, False)
        traffic_manager.auto_lane_change(adv_vehicle, True)
        traffic_manager.distance_to_leading_vehicle(ego_vehicle, 5.0)
        traffic_manager.ignore_vehicles_percentage(ego_vehicle, 100.0)   # TM 자체 앞차 회피 끔 -> 우리 판단기만 안전장치
        traffic_manager.distance_to_leading_vehicle(adv_vehicle, 5.0)
        for vehicle in (ego_vehicle, adv_vehicle):
            traffic_manager.ignore_lights_percentage(vehicle, 100.0)
            traffic_manager.ignore_signs_percentage(vehicle, 100.0)
        set_tm_speed(traffic_manager, ego_vehicle, EGO_TARGET_KMH)
        set_tm_speed(traffic_manager, adv_vehicle, ADV_TARGET_KMH)

        print(f'[INFO] Cut-in 시나리오 시작 (trigger: {TRIGGER_DISTANCE_M}m). Ctrl+C 종료')

        for _ in range(MAX_TICKS):
            world.tick()
            time.sleep(FIXED_DELTA_S)   # 실시간 페이싱 (너무 빨리 진행되지 않게)
            snapshot = world.get_snapshot()
            frame  = snapshot.frame
            time_s = snapshot.timestamp.elapsed_seconds

            if collision_info['detected']:
                result, reason = 'FAIL', f"충돌: {collision_info['other']}"
                break

            # 고정 chase view: 거리 기반 카메라보다 덜 흔들린다.
            world.get_spectator().set_transform(chase_camera_tf(ego_vehicle.get_transform()))
            draw_label(world, ego_vehicle, 'EGO', carla.Color(0, 255, 0))
            draw_label(world, adv_vehicle, 'ADV', carla.Color(255, 80, 80))

            set_tm_speed(traffic_manager, ego_vehicle, EGO_TARGET_KMH)
            set_tm_speed(traffic_manager, adv_vehicle, ADV_CUTIN_KMH if cut_in_triggered else ADV_TARGET_KMH)

            dist_m = ego_vehicle.get_location().distance(adv_vehicle.get_location())
            ego_speed_mps, ego_speed_kmh = get_speed_kmh(ego_vehicle)
            adv_speed_mps, adv_speed_kmh = get_speed_kmh(adv_vehicle)
            accel_mps2 = 0.0 if prev_ego_speed_mps is None else (ego_speed_mps - prev_ego_speed_mps) / delta_seconds
            prev_ego_speed_mps = ego_speed_mps

            # adversary 위치를 Ego 기준 전방/횡방향 성분으로 분해
            ego_transform  = ego_vehicle.get_transform()
            forward_vector = ego_transform.get_forward_vector()
            right_vector   = ego_transform.get_right_vector()
            ego_to_adv = adv_vehicle.get_location() - ego_vehicle.get_location()
            ahead_m   = ego_to_adv.x * forward_vector.x + ego_to_adv.y * forward_vector.y   # 전방(+) 거리 성분
            lateral_m = ego_to_adv.x * right_vector.x   + ego_to_adv.y * right_vector.y     # 횡방향(오른쪽 +) 성분

            ##### TODO 3 #####
            # 위험 대상 판정하기: adversary 가 'Ego 앞' 이고 '내 차선 폭 안' 이면 True
            #   - Ego 앞         : ahead_m > 0
            #   - 내 차선에 걸침 : |lateral_m| < EGO_LANE_HALF   (옆 차선 중앙 3.5 m 는 아님, 넘어오는 중이면 위험)
            # 사용함수: abs(x) / 기능: 절댓값
            #################

            relevant = ...

            #################

            ttc, a_req, gap_m, action = float('inf'), 0.0, float('inf'), 'KEEP'
            if relevant:
                gap_m  = max(ahead_m - VEH_LEN_M, 0.0)
                rel_speed_mps = ego_speed_mps - adv_speed_mps
                ttc    = compute_ttc(gap_m, rel_speed_mps)          # Planning_2 함수
                a_req  = required_decel(gap_m, rel_speed_mps)       # Planning_2 함수
                action = decide(ttc, a_req)                         # Planning_2 함수
                if cut_in_triggered:
                    min_gap_m = min(min_gap_m, gap_m)

            # 판단 -> 제어 (Planning_2 와 같은 방식)
            if action in BRAKE_LEVEL:
                if ego_autopilot:
                    ego_vehicle.set_autopilot(False, tm_port)   # 직접 제동하려면 TM 제어 해제
                    ego_autopilot = False
                ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=lane_keep_steer(world, ego_vehicle),
                                                               brake=BRAKE_LEVEL[action]))
                if action == 'EMERGENCY_BRAKE':
                    emergency_count += 1
                if not ego_braked:
                    ego_braked = True
                    react_gap_m, react_ttc = gap_m, ttc
                    print(f'[REACT] gap={gap_m:.1f}m TTC={ttc:.2f}s 필요감속={a_req:.1f}m/s² -> {action}')
            elif not ego_autopilot:
                # 위험이 막 지났을 때 바로 autopilot 에 넘기면 TM 이 코앞의 차를 보고 급제동
                # -> 간격·TTC 가 충분히 회복될 때까지 약하게 제동 유지 (Planning_2 의 HOLD)
                if (not relevant) or (gap_m > SAFE_GAP_M and ttc > TTC_RESUME):
                    ego_vehicle.set_autopilot(True, tm_port)
                    ego_autopilot = True
                else:
                    action = 'HOLD'
                    ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=lane_keep_steer(world, ego_vehicle),
                                                                   brake=BRAKE_LEVEL['BRAKE']))

            log_rows.append({'frame': frame, 'time_s': round(time_s, 3),
                             'dist_m': round(dist_m, 2), 'gap_m': round(min(gap_m, 999.0), 2),
                             'ego_kmh': round(ego_speed_kmh, 1), 'adv_kmh': round(adv_speed_kmh, 1),
                             'ttc_s': round(min(ttc, 99.0), 2), 'a_req': round(a_req, 2), 'action': action,
                             'accel': round(accel_mps2, 2), 'cutin': int(cut_in_triggered)})

            # Cut-in 트리거
            if not cut_in_triggered and dist_m <= TRIGGER_DISTANCE_M:
                cut_in_triggered = True
                _, target_lane_id = get_lane_info(world, ego_vehicle)
                force_dir_right   = infer_cutin_direction_is_right(world, adv_vehicle, ego_vehicle)

                ##### TODO 4 #####
                # adversary 를 Ego 차선으로 강제 cut-in 시키기 (방향 force_dir_right 는 위에서 이미 추론됨)
                # 사용함수: traffic_manager.force_lane_change(vehicle, force_dir_right) / 기능: 즉시 1회 차선 변경 강제 (True=오른쪽, False=왼쪽)
                #################

                traffic_manager.force_lane_change(adv_vehicle, ...)

                #################
                direction = 'RIGHT' if force_dir_right else 'LEFT'
                print(f'[TRIGGER] dist={dist_m:.1f}m  force_lane_change={direction}')

            # 20틱마다 재요청 (차선 변경 완료 전까지)
            if cut_in_triggered and not cut_in_completed and frame % 20 == 0:
                if get_lane_info(world, adv_vehicle)[1] != target_lane_id:
                    traffic_manager.force_lane_change(adv_vehicle, force_dir_right)

            # Cut-in 완료 판정
            if cut_in_triggered and not cut_in_completed:
                ego_lane = get_lane_info(world, ego_vehicle)
                adv_lane = get_lane_info(world, adv_vehicle)
                if ego_lane[0] is not None and ego_lane == adv_lane:
                    same_lane_hold += 1
                else:
                    same_lane_hold = 0
                if same_lane_hold >= 10:
                    cut_in_completed = True
                    result, reason = 'SUCCESS', 'cut-in 완료'
                    print(f'[SUCCESS] Cut-in 완료 — {POST_CUTIN_SECONDS:.0f}초 더 주행 후 종료')

            # cut-in 완료 후 바로 끝내지 않고 잠깐 더 주행 (갑자기 종료되지 않게)
            if cut_in_completed:
                cooldown_left -= 1
                if cooldown_left <= 0:
                    break
        else:
            result, reason = 'FAIL', f'timeout {MAX_TICKS} ticks'

    except KeyboardInterrupt:
        result, reason = 'STOPPED', 'keyboard interrupt'
        print('\n[Ctrl+C] 종료')
    finally:
        if cut_in_triggered:
            react = f'간격 {react_gap_m:.1f} m, TTC {react_ttc:.2f} s' if ego_braked else '반응 없음'
            # 최대 감속은 출력 안 함: CARLA 제동이 2단계 고정 + 정지 직전 물리값 튐 -> 거리와 무관한 값이 나옴
            print(f'[EVAL] 첫 제동 반응: {react}  |  비상 제동: {emergency_count} tick  |  최소 차간 간격: {min_gap_m:.1f} m')
            print(f'[TABLE] 끼어드는 거리 {TRIGGER_DISTANCE_M:.0f} m | 반응 시점 {react} | 비상 제동 {"O" if emergency_count else "X"} '
                  f'| 최소 간격 {min_gap_m:.1f} m | 충돌 {"O" if collision_info["detected"] else "X"}')
        if log_rows:
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with LOG_PATH.open('w', newline='', encoding='utf-8') as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=list(log_rows[0].keys()))
                writer.writeheader(); writer.writerows(log_rows)
            print(f'[INFO] 로그 저장: {LOG_PATH}')
        client.apply_batch([carla.command.DestroyActor(actor.id) for actor in actor_list])
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print(f'[RESULT] {result}: {reason}')
        print('[DONE] 정리 완료')


if __name__ == '__main__':
    main()
