#!/usr/bin/env python3
#======================================================================#
# Module:      run_scenario_cutin.py
# Description: Town04 고속도로에서 인접 차선 차량이 Ego 앞으로 cut-in 하는 시나리오
#              (carla_template.py / highway_cut_in.py 규약 기반 — 교육용 축약판)
#
# Authors: AILAB (based on Jisang Yun's highway_cut_in.py)
# CARLA 0.9.16
#======================================================================#
import csv
from pathlib import Path

import carla

# ============================================================
# 설정값
# ============================================================
TOWN = 'Town04'
FIXED_DELTA_S = 0.05
EGO_SPAWN_INDEX = 0
ADV_FORWARD_DISTANCE_M = 120.0      # Ego 전방 adversary 스폰 거리
EGO_TARGET_KMH = 50.0
ADV_TARGET_KMH = 30.0
ADV_CUTIN_KMH = 20.0               # cut-in 중 adversary 감속
TRIGGER_DISTANCE_M = 40.0          # 이 거리 안으로 들어오면 cut-in 발동
HARSH_DECEL_MS2 = -4.0             # 급제동 판정 기준
MAX_TICKS = 3000
LOG_PATH = Path('output/run_scenario_cutin_log.csv')


# ============================================================
# Util
# ============================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def get_speed_kmh(vehicle):
    v = vehicle.get_velocity()
    mps = (v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5
    return mps, mps * 3.6


def set_tm_target_speed(tm, vehicle, target_kmh):
    # TM은 제한속도 대비 %로 속도 지정 (음수 = 더 빠르게)
    speed_limit = max(1.0, float(vehicle.get_speed_limit()))
    pct = ((speed_limit - float(target_kmh)) / speed_limit) * 100.0
    tm.vehicle_percentage_speed_difference(vehicle, clamp(pct, -80.0, 95.0))


def set_initial_velocity(vehicle, kmh):
    fwd = vehicle.get_transform().get_forward_vector()
    mps = kmh / 3.6
    vehicle.set_target_velocity(carla.Vector3D(fwd.x * mps, fwd.y * mps, 0.0))


def lane_of(world, vehicle):
    wp = world.get_map().get_waypoint(vehicle.get_location(),
                                      project_to_road=True, lane_type=carla.LaneType.Driving)
    return (wp.road_id, wp.lane_id) if wp else (None, None)


def cutin_to_right(world, adv, ego):
    # adv의 좌/우 차선 중 ego와 같은 lane_id인 쪽으로 끼어들기
    amap = world.get_map()
    awp = amap.get_waypoint(adv.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    ewp = amap.get_waypoint(ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    if awp is None or ewp is None:
        return True
    r, l = awp.get_right_lane(), awp.get_left_lane()
    if r is not None and r.lane_id == ewp.lane_id:
        return True
    if l is not None and l.lane_id == ewp.lane_id:
        return False
    return abs(ewp.lane_id) > abs(awp.lane_id)


def destroy_all_vehicles(world):
    for v in world.get_actors().filter('vehicle.*'):
        v.destroy()


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world(TOWN)
    original_settings = world.get_settings()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_S
    world.apply_settings(settings)
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    tm_port = tm.get_port()

    actor_list = []
    log_rows = []
    collision = {'hit': False, 'other': None}
    cut_in = False
    target_lane = None
    to_right = True
    same_lane_hold = 0
    prev_ego_mps = None
    max_decel = 0.0
    min_dist = float('inf')
    result, reason = 'RUNNING', ''

    try:
        destroy_all_vehicles(world)
        world.tick()

        # Ego spawn
        sps = world.get_map().get_spawn_points()
        ego_bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
        sp = sps[EGO_SPAWN_INDEX]
        ego = world.try_spawn_actor(ego_bp, carla.Transform(
            carla.Location(sp.location.x, sp.location.y, sp.location.z + 0.3), sp.rotation))
        if ego is None:
            raise RuntimeError('ego spawn 실패')
        actor_list.append(ego)
        world.tick()

        # Adversary: ego 전방 인접 차선
        ego_wp = world.get_map().get_waypoint(ego.get_location(), project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        ahead = ego_wp.next(ADV_FORWARD_DISTANCE_M)
        if not ahead:
            raise RuntimeError('전방 waypoint 없음')
        adj = ahead[0].get_right_lane() or ahead[0].get_left_lane()
        if adj is None:
            raise RuntimeError('인접 차선 없음 — 다른 위치/맵에서 재시도')
        adv_bp = world.get_blueprint_library().filter('vehicle.audi.tt')[0]
        adv = world.try_spawn_actor(adv_bp, carla.Transform(
            carla.Location(adj.transform.location.x, adj.transform.location.y,
                           adj.transform.location.z + 0.3), adj.transform.rotation))
        if adv is None:
            raise RuntimeError('adversary spawn 실패')
        actor_list.append(adv)
        world.tick()

        # 충돌 센서
        col_bp = world.get_blueprint_library().find('sensor.other.collision')
        col = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego)
        col.listen(lambda e: collision.update(hit=True, other=e.other_actor.type_id))
        actor_list.append(col)

        # 초기 속도 + autopilot + TM
        set_initial_velocity(ego, EGO_TARGET_KMH)
        set_initial_velocity(adv, ADV_TARGET_KMH)
        ego.set_autopilot(True, tm_port)
        adv.set_autopilot(True, tm_port)
        tm.auto_lane_change(ego, False)
        tm.auto_lane_change(adv, True)
        tm.distance_to_leading_vehicle(ego, 5.0)
        for v in (ego, adv):
            tm.ignore_lights_percentage(v, 100.0)
            tm.ignore_signs_percentage(v, 100.0)
        set_tm_target_speed(tm, ego, EGO_TARGET_KMH)
        set_tm_target_speed(tm, adv, ADV_TARGET_KMH)
        print(f'[INFO] {TOWN} cut-in 시작 (trigger {TRIGGER_DISTANCE_M:.0f}m). Ctrl+C 로 종료')

        for _ in range(MAX_TICKS):
            world.tick()
            snap = world.get_snapshot()
            if collision['hit']:
                result, reason = 'FAIL', f"collision with {collision['other']}"
                break

            # spectator: ego 위에서 진행 방향
            tf = ego.get_transform(); fwd = tf.get_forward_vector()
            world.get_spectator().set_transform(carla.Transform(
                carla.Location(tf.location.x - fwd.x * 6, tf.location.y - fwd.y * 6, tf.location.z + 25),
                carla.Rotation(pitch=-35, yaw=tf.rotation.yaw)))

            set_tm_target_speed(tm, ego, EGO_TARGET_KMH)
            set_tm_target_speed(tm, adv, ADV_CUTIN_KMH if cut_in else ADV_TARGET_KMH)

            dist = ego.get_location().distance(adv.get_location())
            ego_mps, ego_kmh = get_speed_kmh(ego)
            _, adv_kmh = get_speed_kmh(adv)
            accel = 0.0 if prev_ego_mps is None else (ego_mps - prev_ego_mps) / FIXED_DELTA_S
            prev_ego_mps = ego_mps
            if cut_in:
                min_dist = min(min_dist, dist); max_decel = min(max_decel, accel)
            log_rows.append({'frame': snap.frame, 'dist_m': round(dist, 2),
                             'ego_kmh': round(ego_kmh, 1), 'adv_kmh': round(adv_kmh, 1),
                             'ego_accel': round(accel, 2), 'cutin': int(cut_in)})

            # 트리거: 거리 안으로 들어오면 force_lane_change
            if not cut_in and dist <= TRIGGER_DISTANCE_M:
                cut_in = True
                _, target_lane = lane_of(world, ego)
                to_right = cutin_to_right(world, adv, ego)
                tm.force_lane_change(adv, to_right)
                print(f"[TRIGGER] dist={dist:.1f}m force_lane_change={'RIGHT' if to_right else 'LEFT'}")

            # cut-in 중 목표 차선 도달 전까지 20틱마다 재요청
            if cut_in and target_lane is not None and snap.frame % 20 == 0:
                if lane_of(world, adv)[1] != target_lane:
                    tm.force_lane_change(adv, to_right)

            # 완료 판정: 같은 차선 10틱 유지
            if cut_in and result == 'RUNNING':
                if lane_of(world, ego) == lane_of(world, adv):
                    same_lane_hold += 1
                else:
                    same_lane_hold = 0
                if same_lane_hold >= 10:
                    result, reason = 'SUCCESS', 'cut-in completed'
                    print('[RESULT] SUCCESS: cut-in 완료')
                    break
        else:
            result, reason = 'FAIL', f'timeout {MAX_TICKS} ticks'

    except KeyboardInterrupt:
        result, reason = 'STOPPED', 'keyboard interrupt'
        print('\n[Ctrl+C] 종료')
    finally:
        if cut_in:
            print(f'[EVAL] 최대 감속 {max_decel:.2f} m/s^2, 최소 간격 {min_dist:.1f} m')
            if max_decel < HARSH_DECEL_MS2:
                print(f'[WARNING] 급제동 감지 (기준 {HARSH_DECEL_MS2} m/s^2)')
        if log_rows:
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with LOG_PATH.open('w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()))
                w.writeheader(); w.writerows(log_rows)
            print(f'[INFO] 로그 저장: {LOG_PATH}')
        client.apply_batch([carla.command.DestroyActor(a.id) for a in actor_list])
        tm.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print(f'[RESULT] {result}: {reason}')
        print('[INFO] 정리 완료')


if __name__ == '__main__':
    main()
