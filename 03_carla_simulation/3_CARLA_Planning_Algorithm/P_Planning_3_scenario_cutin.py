#!/usr/bin/env python3
#======================================================================#
# Module:      P_Planning_3_scenario_cutin.py
# Description: Cut-in scenario where an adversary cuts in and the ego_vehicle brakes by TTC
#
# Authors: Jisang Yun (jisangyun@hanyang.ac.kr)
#
# Revision History
#      June 28, 2026: Jisang Yun - Created.
#======================================================================#
import csv
import math
import time
from pathlib import Path
import carla

# ============================================================
# 설정값
# ============================================================
# Town04 고속도로 — 인접 차선이 확실히 있는 위치
EGO_SPAWN_TRANSFORM = carla.Transform(
    carla.Location(x=324.159, y=17.343, z=0.808),
    carla.Rotation(pitch=1.553, yaw=-179.022, roll=0.0),
)
ADV_FORWARD_M         = 200.0   # Ego 전방 adversary 스폰 거리
EGO_TARGET_KMH        = 50.0
ADV_TARGET_KMH        = 20.0
ADV_CUTIN_KMH         = 18.0   # cut-in 중 adversary 감속 속도
TRIGGER_DISTANCE_M    = 50.0   # 이 거리 안에 들어오면 cut-in 발동
HARSH_DECEL_THRESHOLD = -4.0   # 급제동 판정 기준 (m/s^2)
MAX_TICKS             = 5000
POST_CUTIN_SECONDS    = 8.0    # cut-in 완료 후 추가 추종 시간
FIXED_DELTA_S         = 0.05
LOG_PATH              = Path('output/cutin_log.csv')

# Ego 의 TTC 반응 임계값 (file 2. TTC 실습과 동일 개념)
TTC_EMERGENCY = 1.5   # (s) 이하 → 비상 제동 (full brake)
TTC_BRAKE     = 3.0   # (s) 이하 → 제동
VEH_LEN_M     = 4.5   # 차량 길이 보정 (gap 계산용)
EGO_LANE_HALF = 4.0   # adversary 가 'Ego 앞'으로 볼 횡방향 허용폭 (m)

# ============================================================
# Util
# ============================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def get_speed_kmh(vehicle):
    v = vehicle.get_velocity()
    mps = math.sqrt(v.x**2 + v.y**2 + v.z**2)
    return mps, mps * 3.6

def compute_ttc(gap_m, rel_speed_mps):
    #-[TODO]- Implement TTC = gap / rel_speed (return inf if rel_speed <= 0, i.e. moving apart)
    #         Same concept as the TTC computed in practice 2.
    #[key functions] simple division, boundary check
    if rel_speed_mps <=
        return float('inf')
    return gap_m /

def set_tm_speed(traffic_manager, vehicle, target_kmh):
    limit = max(1.0, float(vehicle.get_speed_limit()))
    pct   = ((limit - float(target_kmh)) / limit) * 100.0
    traffic_manager.vehicle_percentage_speed_difference(vehicle, clamp(pct, -80.0, 95.0))

def get_lane_info(world, vehicle):
    wp = world.get_map().get_waypoint(
        vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    return (wp.road_id, wp.lane_id) if wp else (None, None)

def draw_label(world, actor, text, color, dt):
    world.debug.draw_string(
        actor.get_location() + carla.Location(z=2.5),
        text, draw_shadow=False, color=color, life_time=dt, persistent_lines=False)

# ============================================================
# cut-in 방향 추론
# ============================================================
def infer_cutin_direction_is_right(world, adv_vehicle, ego_vehicle):
    amap   = world.get_map()
    adv_wp = amap.get_waypoint(adv_vehicle.get_location(),
                               project_to_road=True, lane_type=carla.LaneType.Driving)
    ego_wp = amap.get_waypoint(ego_vehicle.get_location(),
                               project_to_road=True, lane_type=carla.LaneType.Driving)
    if adv_wp is None or ego_wp is None:
        return True
    right_wp = adv_wp.get_right_lane()
    left_wp  = adv_wp.get_left_lane()
    if right_wp is not None and right_wp.lane_id == ego_wp.lane_id:
        return True
    if left_wp is not None and left_wp.lane_id == ego_wp.lane_id:
        return False
    return abs(ego_wp.lane_id) > abs(adv_wp.lane_id)

# ============================================================
# Spectator
# ============================================================
def chase_camera_tf(ego_transform):
    # force_lane_change 파일과 같은 스타일의 고정 chase view
    offset = carla.Location(x=-8.0, y=0.0, z=4.0)
    loc = ego_transform.transform(offset)
    rot = carla.Rotation(roll=0.0, pitch=-15.0, yaw=ego_transform.rotation.yaw)
    return carla.Transform(loc, rot)

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
    dt = settings.fixed_delta_seconds

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    tm_port = traffic_manager.get_port()

    actor_list    = []
    log_rows      = []
    collision_info = {'detected': False, 'other': None}

    cut_in_triggered = False
    cut_in_completed = False
    target_lane_id   = None
    force_dir_right  = True
    same_lane_hold   = 0
    prev_ego_mps     = None
    max_decel        = 0.0
    min_dist         = float('inf')
    cooldown_left    = int(POST_CUTIN_SECONDS / FIXED_DELTA_S)
    ego_autopilot    = True            # Ego 는 평소 autopilot, TTC 위험 시 직접 제동
    ego_braked       = False           # 한 번이라도 TTC 제동했는지 (평가용)
    result, reason   = 'RUNNING', ''

    try:
        # 기존 차량 정리
        for v in world.get_actors().filter('vehicle.*'):
            v.destroy()
        world.tick()

        # ============================================================
        # Ego Spawn
        # ============================================================
        ego_bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
        ego_bp.set_attribute('role_name', 'ego')
        ego_bp.set_attribute('color', '255,0,0')
        ego_vehicle = world.try_spawn_actor(ego_bp, carla.Transform(
            carla.Location(EGO_SPAWN_TRANSFORM.location.x,
                           EGO_SPAWN_TRANSFORM.location.y,
                           EGO_SPAWN_TRANSFORM.location.z + 0.3),
            EGO_SPAWN_TRANSFORM.rotation))
        if ego_vehicle is None:
            raise RuntimeError('ego spawn 실패')
        actor_list.append(ego_vehicle)
        print(f'[SPAWN] Ego at ({EGO_SPAWN_TRANSFORM.location.x:.1f}, {EGO_SPAWN_TRANSFORM.location.y:.1f})')
        world.tick()

        # ============================================================
        # Adversary: Ego 전방 인접 차선
        # ============================================================
        ego_wp    = world.get_map().get_waypoint(
            ego_vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)

        #-[TODO]- Get the forward waypoints ADV_FORWARD_M ahead of the ego
        #[key functions] wp.next(distance) -> list[Waypoint]
        ahead_wps = ego_wp.next(
        if not ahead_wps:
            raise RuntimeError('전방 waypoint 없음')

        #-[TODO]- Get the adjacent-lane waypoint with get_right_lane() or get_left_lane()
        #[key functions] wp.get_right_lane(), wp.get_left_lane()
        adj_wp = ahead_wps[0].get_right_lane() or ahead_wps[0].
        if adj_wp is None:
            raise RuntimeError('인접 차선 없음')

        adv_bp = world.get_blueprint_library().filter('vehicle.audi.tt')[0]
        adv_vehicle    = world.try_spawn_actor(adv_bp, carla.Transform(
            carla.Location(adj_wp.transform.location.x,
                           adj_wp.transform.location.y,
                           adj_wp.transform.location.z + 0.3),
            adj_wp.transform.rotation))
        if adv_vehicle is None:
            raise RuntimeError('adversary spawn 실패 — ADV_FORWARD_M 값을 바꿔보세요')
        actor_list.append(adv_vehicle)
        print(f'[SPAWN] Adversary {ADV_FORWARD_M:.0f}m 전방 인접 차선')
        world.tick()

        # ============================================================
        # 충돌 센서
        # ============================================================
        col_bp = world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor    = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego_vehicle)
        collision_sensor.listen(lambda e: collision_info.update(detected=True, other=e.other_actor.type_id))
        actor_list.append(collision_sensor)

        # ============================================================
        # 초기 속도 + Autopilot + TM 설정
        # ============================================================
        ego_vehicle.set_autopilot(True, tm_port)
        adv_vehicle.set_autopilot(True, tm_port)
        traffic_manager.auto_lane_change(ego_vehicle, False)
        traffic_manager.auto_lane_change(adv_vehicle, True)
        traffic_manager.distance_to_leading_vehicle(ego_vehicle, 5.0)
        traffic_manager.distance_to_leading_vehicle(adv_vehicle, 5.0)
        for v in (ego_vehicle, adv_vehicle):
            traffic_manager.ignore_lights_percentage(v, 100.0)
            traffic_manager.ignore_signs_percentage(v, 100.0)
        set_tm_speed(traffic_manager, ego_vehicle, EGO_TARGET_KMH)
        set_tm_speed(traffic_manager, adv_vehicle, ADV_TARGET_KMH)

        print(f'[INFO] Cut-in 시나리오 시작 (trigger: {TRIGGER_DISTANCE_M}m). Ctrl+C 종료')

        for _ in range(MAX_TICKS):
            world.tick()
            time.sleep(FIXED_DELTA_S)   # 실시간 페이싱 (너무 빨리 진행되지 않게)
            snap  = world.get_snapshot()
            frame = snap.frame
            time_s = snap.timestamp.elapsed_seconds

            if collision_info['detected']:
                result, reason = 'FAIL', f"충돌: {collision_info['other']}"
                break

            # 고정 chase view: 거리 기반 카메라보다 덜 흔들린다.
            world.get_spectator().set_transform(chase_camera_tf(ego_vehicle.get_transform()))
            draw_label(world, ego_vehicle, 'EGO', carla.Color(0, 255, 0), dt)
            draw_label(world, adv_vehicle, 'ADV', carla.Color(255, 80, 80), dt)

            set_tm_speed(traffic_manager, ego_vehicle, EGO_TARGET_KMH)
            set_tm_speed(traffic_manager, adv_vehicle, ADV_CUTIN_KMH if cut_in_triggered else ADV_TARGET_KMH)

            dist = ego_vehicle.get_location().distance(adv_vehicle.get_location())
            ego_mps, ego_kmh = get_speed_kmh(ego_vehicle)
            adv_mps, adv_kmh = get_speed_kmh(adv_vehicle)
            accel = 0.0 if prev_ego_mps is None else (ego_mps - prev_ego_mps) / dt
            prev_ego_mps = ego_mps

            if cut_in_triggered and not cut_in_completed:
                min_dist  = min(min_dist, dist)
                max_decel = min(max_decel, accel)

            # Ego 의 TTC 반응 (file 2 에서 배운 판단을 여기에 적용)
            # adversary 가 'Ego 앞 + 같은 차선 폭 이내'에 있을 때만 충돌위험으로 본다.
            ego_tf = ego_vehicle.get_transform()
            fwd, rgt = ego_tf.get_forward_vector(), ego_tf.get_right_vector()
            to_adv = adv_vehicle.get_location() - ego_vehicle.get_location()
            ahead   = to_adv.x * fwd.x + to_adv.y * fwd.y     # 전방(+) 거리 성분
            lateral = to_adv.x * rgt.x + to_adv.y * rgt.y     # 횡방향 오프셋
            relevant = ahead > 0 and abs(lateral) < EGO_LANE_HALF

            ttc = float('inf')
            if relevant:
                gap = max(ahead - VEH_LEN_M, 0.0)
                ttc = compute_ttc(gap, ego_mps - adv_mps)

            #-[TODO]- If TTC is dangerous, turn off autopilot and brake directly; otherwise resume autopilot
            #         ttc < TTC_EMERGENCY -> full brake(1.0), ttc < TTC_BRAKE -> brake(0.6)
            #[key functions] ego_vehicle.set_autopilot(bool, tm_port), ego_vehicle.apply_control(VehicleControl)
            if relevant and ttc < TTC_BRAKE:
                if ego_autopilot:
                    ego_vehicle.set_autopilot(False, tm_port)
                    ego_autopilot = False
                #-[TODO]- Choose the brake strength according to the TTC threshold.
                #[key functions] ttc < TTC_EMERGENCY -> 1.0, otherwise 0.6
                brake_val = 1.0 if ttc <
                ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=brake_val))
                if not ego_braked:
                    ego_braked = True
                    print(f'[REACT] TTC={ttc:.2f}s → Ego 제동 (brake={brake_val})')
            else:
                if not ego_autopilot:
                    ego_vehicle.set_autopilot(True, tm_port)
                    ego_autopilot = True

            log_rows.append({'frame': frame, 'time_s': round(time_s, 3),
                             'dist_m': round(dist, 2),
                             'ego_kmh': round(ego_kmh, 1), 'adv_kmh': round(adv_kmh, 1),
                             'ttc_s': round(min(ttc, 99.0), 2),
                             'accel': round(accel, 2), 'cutin': int(cut_in_triggered)})

            # Cut-in 트리거
            if not cut_in_triggered and dist <= TRIGGER_DISTANCE_M:
                cut_in_triggered = True
                _, target_lane_id = get_lane_info(world, ego_vehicle)
                force_dir_right   = infer_cutin_direction_is_right(world, adv_vehicle, ego_vehicle)

                #-[TODO]- Force the adversary to cut in with traffic_manager.force_lane_change()
                #[key functions] traffic_manager.force_lane_change(vehicle, to_right: bool)
                traffic_manager.force_lane_change(
                direction = 'RIGHT' if force_dir_right else 'LEFT'
                print(f'[TRIGGER] dist={dist:.1f}m  force_lane_change={direction}')

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
            print(f'[EVAL] 최소 간격: {min_dist:.1f}m  |  최대 감속: {max_decel:.2f} m/s²')
            print(f'[EVAL] Ego TTC 제동 반응: {"있었음" if ego_braked else "없었음"}')
            if max_decel < HARSH_DECEL_THRESHOLD:
                print(f'[WARNING] 급제동 감지! (기준 {HARSH_DECEL_THRESHOLD} m/s²)')
        if log_rows:
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with LOG_PATH.open('w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()))
                w.writeheader(); w.writerows(log_rows)
            print(f'[INFO] 로그 저장: {LOG_PATH}')
        client.apply_batch([carla.command.DestroyActor(a.id) for a in actor_list])
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print(f'[RESULT] {result}: {reason}')
        print('[DONE] 정리 완료')


if __name__ == '__main__':
    main()
