#!/usr/bin/env python3
#======================================================================#
# Module:      P_Planning_2_ttc_braking.py
# Description: 판단(Behavior): 앞차 위험도(TTC + 필요 감속도)로 제동 여부·세기 결정
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      June 28, 2026: Jisang Yun - Created. (P_Planning_2_ttc_decision)
#      Oct  19, 2026: Jisang Yun - 필요 감속도 추가, Day 2 감속 한계(a_min) 연결
#======================================================================#
# 판단에 쓰는 두 값
#   TTC(Time To Collision, 충돌까지 남은 시간) = 간격 / 접근 속도  -> '얼마나 빨리' 부딪히나
#   필요 감속도 = 접근 속도² / (2 · 간격)  -> 앞차 속도까지 '얼마나 세게' 줄여야 하나
#                (Day 2 공식 v² = v0² + 2·a·ds 에서 v0 = 접근 속도, v = 0(앞차와 같은 속도))
# 같은 TTC 라도 접근 속도가 크면 필요 감속도가 큼 -> TTC 만으로는 비상 여부를 알 수 없음
#   예) TTC 2.2 s 로 같아도  자차 55 / 앞차 5 km/h,  간격 30 m : 3.2 m/s² (비상)
#                           자차 30 / 앞차 25 km/h, 간격 3 m  : 0.3 m/s² (일반 제동)
# 판단 기준 A_COMFORT = 2.5 m/s² = Day 2 Step02 의 감속 한계 a_min(-2.5) 과 같은 값
#======================================================================#
import csv
import math
import time
from pathlib import Path
import carla

# ============================================================
# 설정값
# ============================================================
TOWN              = 'Town04'
FIXED_DELTA_S     = 0.05
EGO_SPAWN_INDEX   = 0
LEAD_FORWARD_M    = 30.0    # Ego 전방 선행차 초기 거리
EGO_TARGET_KMH    = 45.0    # 선행차와 같은 속도 -> 급정거 전까지 간격 유지
LEAD_TARGET_KMH   = 45.0
HARD_BRAKE_S      = 6.0     # 시작 후 몇 초 뒤 선행차 급정거 (brake 1.0, 약 7.6 m/s²). 앞 6초는 두 차 속도 안정화
STOP_HOLD_S       = 2.0     # 두 차가 이 시간 동안 정지해 있으면 종료
MAX_TICKS         = 600
LOG_PATH          = Path('output/ttc_log.csv')

# 판단 임계값
A_COMFORT     = 2.5   # (m/s²) 편안한 감속 한계 (Day 2 a_min) -> 필요 감속도가 이보다 크면 비상 제동
TTC_BRAKE     = 3.0   # (s) 미만 -> 제동
TTC_CAUTION   = 5.0   # (s) 미만 -> 주의 (속도 유지, 상황 주시)
TTC_RESUME    = 6.0   # (s) 이상 + SAFE_GAP_M 이상일 때만 제동을 풀고 autopilot 복귀
SAFE_GAP_M    = 15.0  # (m) 제동 해제용 최소 간격
# 판단 -> brake 값 (CARLA 차량은 throttle 0 만으로도 약 4.5 m/s² 감속 -> brake 값과 감속도가 비례하지 않아 단계로 지정)
BRAKE_LEVEL   = {'BRAKE': 0.3, 'EMERGENCY_BRAKE': 1.0}

# ============================================================
# TTC 계산
# ============================================================
def compute_ttc(gap_m, rel_speed_mps):
    # rel_speed_mps = ego 속도 - lead 속도 (양수면 선행차에 접근 중이라는 뜻)
    ##### TODO 1 #####
    # TTC 계산하기: 접근 중 아님(rel_speed_mps <= 0) -> inf, 접근 중 -> 간격 / 접근 속도
    #################

    if rel_speed_mps <= ...:
        return float('inf')
    return ...

    #################

# ============================================================
# 필요 감속도: 지금 간격 안에서 앞차 속도까지 줄이려면 필요한 감속도
# ============================================================
def required_decel(gap_m, rel_speed_mps):
    # rel_speed_mps = ego 속도 - lead 속도 (양수 = 접근 중)
    ##### TODO 2 #####
    # 필요 감속도 계산하기: a_req = 접근 속도² / (2 · 간격)
    #   - 접근 중이 아니면 (rel_speed_mps <= 0) 감속 불필요 -> 0.0
    #   - 간격 0 나눗셈 방지: gap 은 max(gap_m, 0.1) 사용
    #################

    if rel_speed_mps <= ...:
        return 0.0
    return ...

    #################

# ============================================================
# 판단: TTC + 필요 감속도 -> 행동
# ============================================================
def decide(ttc, a_req):
    ##### TODO 3 #####
    # 위급한 순서대로 판단하기 (위에서부터 처음 맞는 조건 반환)
    #   - a_req > A_COMFORT -> 'EMERGENCY_BRAKE'  (편안한 감속으로는 못 멈춤)
    #   - ttc < TTC_BRAKE   -> 'BRAKE'
    #   - ttc < TTC_CAUTION -> 'CAUTION'
    #   - 그 외             -> 'KEEP'
    # 확인: main() 시작 시 예시 2개 자동 검사 (TTC 같아도 판단이 달라야 정답)
    #################

    if a_req > ...:
        return 'EMERGENCY_BRAKE'
    if ttc < ...:
        return 'BRAKE'
    if ttc < ...:
        return 'CAUTION'
    return 'KEEP'

    #################

def self_check():
    # 판단 자가 확인 (CARLA 서버 없이도 확인 가능): 둘 다 TTC 약 2.2 s
    cases = [('A. 자차 55 / 앞차 5 km/h, 간격 30 m', 55.0, 5.0, 30.0, 'EMERGENCY_BRAKE'),
             ('B. 자차 30 / 앞차 25 km/h, 간격 3 m', 30.0, 25.0, 3.0, 'BRAKE')]
    for name, v_ego_kmh, v_lead_kmh, gap_m, expected in cases:
        rel = (v_ego_kmh - v_lead_kmh) / 3.6
        ttc, a_req = compute_ttc(gap_m, rel), required_decel(gap_m, rel)
        action = decide(ttc, a_req)
        print(f'[CHECK] {name}: TTC {ttc:.1f} s, 필요 감속도 {a_req:.1f} m/s² -> {action}')
        assert action == expected, f'판단 오답: {name} 기대값 {expected} -> TODO 2 (required_decel) / TODO 3 (decide) 확인'
    assert compute_ttc(10.0, -1.0) == float('inf') and required_decel(10.0, -1.0) == 0.0, \
        'TODO 1·2 확인 - 멀어지는 중(rel <= 0)이면 TTC = inf, 필요 감속도 = 0'

# ============================================================
# Util
# ============================================================
def get_speed_mps(vehicle):
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

def clamp(value, low, high):
    return max(low, min(high, value))

def set_tm_speed(traffic_manager, vehicle, target_kmh):
    speed_limit = max(1.0, float(vehicle.get_speed_limit()))
    # speed_diff_pct: 제한속도 대비 몇 % 느리게(+)/빠르게(-) 달릴지
    speed_diff_pct = ((speed_limit - float(target_kmh)) / speed_limit) * 100.0
    traffic_manager.vehicle_percentage_speed_difference(vehicle, clamp(speed_diff_pct, -80.0, 100.0))   # 100% = 정지

def lane_keep_steer(world, vehicle, lookahead_m=8.0):
    # 직접 제동하는 동안(autopilot OFF)에도 차선 유지: 차선 중앙 lookahead_m 앞 점을 향해 조향
    # (steer 를 안 주면 0 = 핸들을 놓은 상태 -> 곡선 구간에서 차선 이탈·흔들림)
    transform = vehicle.get_transform()
    waypoint  = world.get_map().get_waypoint(transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    ahead     = waypoint.next(lookahead_m)
    target    = (ahead[0] if ahead else waypoint).transform.location
    forward, right = transform.get_forward_vector(), transform.get_right_vector()
    dx, dy = target.x - transform.location.x, target.y - transform.location.y
    local_ahead = dx * forward.x + dy * forward.y
    local_side  = dx * right.x + dy * right.y
    return clamp(local_side / max(abs(local_ahead), 1.0), -1.0, 1.0)   # +: 우회전

def set_initial_velocity(vehicle, kmh):
    forward_vector = vehicle.get_transform().get_forward_vector()
    speed_mps = kmh / 3.6
    vehicle.set_target_velocity(carla.Vector3D(forward_vector.x * speed_mps, forward_vector.y * speed_mps, 0.0))

# ============================================================
# Main
# ============================================================
def main():
    self_check()

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world(TOWN)
    original_settings = world.get_settings()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_S
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    tm_port = traffic_manager.get_port()   # tm_port: Traffic Manager 포트 번호

    actor_list = []
    history    = []
    collision  = {'hit': False, 'other': None}
    ego_autopilot = True
    brake_hold = False
    hard_brake_tick = int(HARD_BRAKE_S / FIXED_DELTA_S)
    stopped_ticks   = 0
    max_decel, ego_speeds = 0.0, []   # 최대 감속: 0.5 s 구간 평균 (한 tick 값은 CARLA 물리값이 튐)
    emergency_count = 0

    try:
        for vehicle in world.get_actors().filter('vehicle.*'):
            vehicle.destroy()
        world.tick()

        spawn_points = world.get_map().get_spawn_points()

        # ============================================================
        # Ego Spawn
        # ============================================================
        ego_blueprint = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_blueprint.set_attribute('color', '255,0,0')
        ego_spawn_point = spawn_points[EGO_SPAWN_INDEX]
        ego_vehicle = world.try_spawn_actor(ego_blueprint, carla.Transform(
            carla.Location(ego_spawn_point.location.x, ego_spawn_point.location.y, ego_spawn_point.location.z + 0.3),
            ego_spawn_point.rotation))
        if ego_vehicle is None:
            raise RuntimeError('ego spawn 실패')
        actor_list.append(ego_vehicle)
        world.tick()

        # ============================================================
        # 선행차 Spawn (ego_vehicle 전방 같은 차선)
        # ============================================================
        ego_waypoint = world.get_map().get_waypoint(
            ego_vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)

        ##### TODO 4 #####
        # 차선을 따라 Ego 전방 LEAD_FORWARD_M 만큼 앞의 지점(선행차 스폰 위치) 구하기
        # 사용함수: ego_waypoint.next(distance) / 기능: 차선을 따라 distance(m) 앞의 waypoint 목록 반환 (list[Waypoint])
        #################

        lead_waypoints = ...

        #################
        if not lead_waypoints:
            raise RuntimeError('선행차 spawn 위치 없음')
        lead_transform = lead_waypoints[0].transform

        lead_blueprint = world.get_blueprint_library().find('vehicle.audi.tt')
        lead_vehicle = world.try_spawn_actor(lead_blueprint, carla.Transform(
            carla.Location(lead_transform.location.x, lead_transform.location.y, lead_transform.location.z + 0.3),
            lead_transform.rotation))
        if lead_vehicle is None:
            raise RuntimeError('선행차 spawn 실패')
        actor_list.append(lead_vehicle)
        world.tick()

        # ============================================================
        # 충돌 센서
        # ============================================================
        collision_blueprint = world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor = world.spawn_actor(collision_blueprint, carla.Transform(), attach_to=ego_vehicle)
        collision_sensor.listen(lambda event: collision.update(hit=True, other=event.other_actor.type_id))
        actor_list.append(collision_sensor)

        # ============================================================
        # 초기 속도 + Autopilot
        # ============================================================
        set_initial_velocity(ego_vehicle, EGO_TARGET_KMH)
        set_initial_velocity(lead_vehicle, LEAD_TARGET_KMH)
        ego_vehicle.set_autopilot(True, tm_port)
        lead_vehicle.set_autopilot(True, tm_port)
        traffic_manager.auto_lane_change(ego_vehicle, False)
        traffic_manager.auto_lane_change(lead_vehicle, False)
        for vehicle in (ego_vehicle, lead_vehicle):
            traffic_manager.ignore_lights_percentage(vehicle, 100.0)
        traffic_manager.ignore_vehicles_percentage(ego_vehicle, 100.0)   # TM 자체 앞차 회피 끔 -> 우리 판단기만 안전장치
        set_tm_speed(traffic_manager, ego_vehicle, EGO_TARGET_KMH)
        set_tm_speed(traffic_manager, lead_vehicle, LEAD_TARGET_KMH)

        print(f'[INFO] 앞차 급정거 시나리오 시작 ({HARD_BRAKE_S}s 후 선행차 급정거)')
        print(f'       판단 기준: 필요 감속도>{A_COMFORT} m/s² -> EMERGENCY / TTC<{TTC_BRAKE}s -> BRAKE / TTC<{TTC_CAUTION}s -> CAUTION')
        spectator = world.get_spectator()

        for tick in range(MAX_TICKS):
            world.tick()
            time.sleep(FIXED_DELTA_S)   # 실시간 페이싱 (너무 빨리 진행되지 않게)
            elapsed_s = tick * FIXED_DELTA_S

            if collision['hit']:
                print(f'[FAIL] 충돌 발생: {collision["other"]}')
                break

            # spectator 업데이트 (ego_vehicle 뒤 chase view)
            ego_transform  = ego_vehicle.get_transform()
            forward_vector = ego_transform.get_forward_vector()
            spectator.set_transform(carla.Transform(
                carla.Location(ego_transform.location.x - forward_vector.x*8,
                               ego_transform.location.y - forward_vector.y*8,
                               ego_transform.location.z + 5),
                carla.Rotation(pitch=-15, yaw=ego_transform.rotation.yaw)))

            # 선행차 급정거 (트리거 시점 이후 계속 brake 1.0)
            if tick == hard_brake_tick:
                lead_vehicle.set_autopilot(False, tm_port)
                print(f'  t={elapsed_s:4.1f}s  [EVENT] 선행차 급정거!')
            if tick >= hard_brake_tick:
                lead_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

            # TTC 계산
            gap_m          = max(ego_vehicle.get_location().distance(lead_vehicle.get_location()) - 4.5, 0.0)
            ego_speed_mps  = get_speed_mps(ego_vehicle)
            lead_speed_mps = get_speed_mps(lead_vehicle)

            ego_speeds.append(ego_speed_mps)
            window = int(0.5 / FIXED_DELTA_S)
            if tick > hard_brake_tick and len(ego_speeds) > window and min(ego_speeds[-window - 1:]) > 1.0:
                max_decel = min(max_decel, (ego_speeds[-1] - ego_speeds[-window - 1]) / (window * FIXED_DELTA_S))

            # 위에서 구현한 함수로 TTC·필요 감속도 계산 → 행동 결정
            rel_speed_mps = ego_speed_mps - lead_speed_mps
            ttc    = compute_ttc(gap_m, rel_speed_mps)
            a_req  = required_decel(gap_m, rel_speed_mps)
            action = decide(ttc, a_req)

            # 판단 결과 → Ego 제어
            if action in ('BRAKE', 'EMERGENCY_BRAKE'):
                if ego_autopilot:
                    ego_vehicle.set_autopilot(False, tm_port)
                    ego_autopilot = False
                brake_hold = True
                brake_value = BRAKE_LEVEL[action]
                steer = lane_keep_steer(world, ego_vehicle)
                emergency_count += action == 'EMERGENCY_BRAKE'
                ##### TODO 5 #####
                # 제동 명령 보내기: throttle 0, brake = 위에서 정한 brake_value
                #   - steer 는 차선 유지값 (바로 아래 steer 변수, 0 이면 핸들을 놓은 상태)
                # 사용함수: carla.VehicleControl(throttle=, steer=, brake=) / 기능: 차량 제어 명령 생성 (throttle=가속 0~1, brake=제동 0~1)
                # 주의: 위치 인자 순서 = (throttle, steer, brake, ...) -> 2번째는 steer, 이름=값 으로 작성
                #################

                ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=..., brake=...))

                #################
            else:
                if brake_hold:
                    if gap_m > SAFE_GAP_M and ttc > TTC_RESUME:
                        brake_hold = False
                        ego_vehicle.set_autopilot(True, tm_port)
                        ego_autopilot = True
                    else:
                        action = 'HOLD'   # 위험은 지났지만 간격·TTC 회복 전까지 제동 유지
                        ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=lane_keep_steer(world, ego_vehicle),
                                                                       brake=BRAKE_LEVEL['BRAKE']))
                elif not ego_autopilot:
                    ego_vehicle.set_autopilot(True, tm_port)
                    ego_autopilot = True

            # 로그 기록 및 출력
            ttc_display = min(ttc, 10.0)
            history.append((round(elapsed_s, 2), round(gap_m, 2), round(ttc_display, 2), round(a_req, 2), action))

            if tick % 10 == 0 or (tick >= hard_brake_tick and action != history[-2][-1]):
                print(f'  t={elapsed_s:4.1f}s  gap={gap_m:5.1f}m  ego={ego_speed_mps * 3.6:4.0f}km/h  lead={lead_speed_mps * 3.6:4.0f}km/h'
                      f'  TTC={ttc_display:5.2f}s  필요감속={a_req:4.1f}m/s²  → {action}')

            # 두 차 모두 정지 -> 종료
            stopped_ticks = stopped_ticks + 1 if tick > hard_brake_tick and ego_speed_mps < 0.1 and lead_speed_mps < 0.1 else 0
            if stopped_ticks * FIXED_DELTA_S >= STOP_HOLD_S:
                print(f'[SUCCESS] 충돌 없이 정지 | 최종 간격 {gap_m:.1f} m | 최대 감속 {max_decel:.1f} m/s² '
                      f'(편안한 한계 -{A_COMFORT}) | 비상 제동 {emergency_count} tick')
                break
        else:
            print('[INFO] 시간 초과로 종료')

    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        # 로그 저장
        if history:
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with LOG_PATH.open('w', newline='', encoding='utf-8') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(['time_s', 'gap_m', 'ttc_s', 'a_req_mps2', 'action'])
                writer.writerows(history)
            print(f'[INFO] 로그 저장: {LOG_PATH}')

        client.apply_batch([carla.command.DestroyActor(actor.id) for actor in actor_list])
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print('[DONE] 정리 완료')


if __name__ == '__main__':
    main()
