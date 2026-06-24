#!/usr/bin/env python3
#======================================================================#
# Module:      run_scenario_decel.py
# Description: 같은 차선 선행차(lead)가 급감속하는 시나리오 + Ego 반응 평가
#              (carla_template.py 규약 기반)
#
# Authors: AILAB (based on Jisang Yun's carla_template.py)
# CARLA 0.9.16
#======================================================================#
import csv
from pathlib import Path

import carla

TOWN = 'Town04'
FIXED_DELTA_S = 0.05
EGO_SPAWN_INDEX = 0
LEAD_FORWARD_M = 25.0
EGO_TARGET_KMH = 50.0
LEAD_TARGET_KMH = 45.0
LEAD_DECEL_KMH = 5.0               # 급감속 후 목표 속도
DECEL_TRIGGER_TICK = 80            # 이 틱부터 lead 급감속
HARSH_DECEL_MS2 = -4.0
MAX_TICKS = 2400
LOG_PATH = Path('output/run_scenario_decel_log.csv')


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def get_speed_kmh(vehicle):
    v = vehicle.get_velocity()
    mps = (v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5
    return mps, mps * 3.6


def set_tm_target_speed(tm, vehicle, target_kmh):
    speed_limit = max(1.0, float(vehicle.get_speed_limit()))
    pct = ((speed_limit - float(target_kmh)) / speed_limit) * 100.0
    tm.vehicle_percentage_speed_difference(vehicle, clamp(pct, -80.0, 95.0))


def set_initial_velocity(vehicle, kmh):
    fwd = vehicle.get_transform().get_forward_vector()
    mps = kmh / 3.6
    vehicle.set_target_velocity(carla.Vector3D(fwd.x * mps, fwd.y * mps, 0.0))


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
    prev_ego_mps = None
    max_decel = 0.0
    min_dist = float('inf')
    result, reason = 'RUNNING', ''

    try:
        destroy_all_vehicles(world)
        world.tick()

        sps = world.get_map().get_spawn_points()
        sp = sps[EGO_SPAWN_INDEX]
        ego = world.try_spawn_actor(world.get_blueprint_library().filter('vehicle.tesla.model3')[0],
                                    carla.Transform(carla.Location(sp.location.x, sp.location.y, sp.location.z + 0.3), sp.rotation))
        if ego is None:
            raise RuntimeError('ego spawn 실패')
        actor_list.append(ego)
        world.tick()

        ego_wp = world.get_map().get_waypoint(ego.get_location(), project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        ahead = ego_wp.next(LEAD_FORWARD_M)
        if not ahead:
            raise RuntimeError('전방 waypoint 없음')
        ltf = ahead[0].transform
        lead = world.try_spawn_actor(world.get_blueprint_library().filter('vehicle.audi.tt')[0],
                                     carla.Transform(carla.Location(ltf.location.x, ltf.location.y, ltf.location.z + 0.3), ltf.rotation))
        if lead is None:
            raise RuntimeError('lead spawn 실패')
        actor_list.append(lead)
        world.tick()

        col_bp = world.get_blueprint_library().find('sensor.other.collision')
        col = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego)
        col.listen(lambda e: collision.update(hit=True, other=e.other_actor.type_id))
        actor_list.append(col)

        set_initial_velocity(ego, EGO_TARGET_KMH)
        set_initial_velocity(lead, LEAD_TARGET_KMH)
        ego.set_autopilot(True, tm_port)
        lead.set_autopilot(True, tm_port)
        tm.auto_lane_change(ego, False)
        tm.auto_lane_change(lead, False)
        tm.distance_to_leading_vehicle(ego, 5.0)
        for v in (ego, lead):
            tm.ignore_lights_percentage(v, 100.0)
            tm.ignore_signs_percentage(v, 100.0)
        set_tm_target_speed(tm, ego, EGO_TARGET_KMH)
        set_tm_target_speed(tm, lead, LEAD_TARGET_KMH)
        print(f'[INFO] {TOWN} 선행차 급감속 시나리오 시작. Ctrl+C 로 종료')

        decel_started = False
        for tick in range(MAX_TICKS):
            world.tick()
            snap = world.get_snapshot()
            if collision['hit']:
                result, reason = 'FAIL', f"collision with {collision['other']}"
                break

            tf = ego.get_transform(); fwd = tf.get_forward_vector()
            world.get_spectator().set_transform(carla.Transform(
                carla.Location(tf.location.x - fwd.x * 8, tf.location.y - fwd.y * 8, tf.location.z + 5),
                carla.Rotation(pitch=-15, yaw=tf.rotation.yaw)))

            # lead 급감속 트리거
            if tick >= DECEL_TRIGGER_TICK:
                set_tm_target_speed(tm, lead, LEAD_DECEL_KMH)
                if not decel_started:
                    decel_started = True
                    print(f'[TRIGGER] tick={tick} lead 급감속 시작 -> {LEAD_DECEL_KMH}km/h')
            set_tm_target_speed(tm, ego, EGO_TARGET_KMH)

            dist = ego.get_location().distance(lead.get_location())
            ego_mps, ego_kmh = get_speed_kmh(ego)
            _, lead_kmh = get_speed_kmh(lead)
            accel = 0.0 if prev_ego_mps is None else (ego_mps - prev_ego_mps) / FIXED_DELTA_S
            prev_ego_mps = ego_mps
            if decel_started:
                min_dist = min(min_dist, dist); max_decel = min(max_decel, accel)
            log_rows.append({'frame': snap.frame, 'dist_m': round(dist, 2),
                             'ego_kmh': round(ego_kmh, 1), 'lead_kmh': round(lead_kmh, 1),
                             'ego_accel': round(accel, 2)})
        else:
            result, reason = 'SUCCESS', 'no collision (timeout)'

    except KeyboardInterrupt:
        result, reason = 'STOPPED', 'keyboard interrupt'
        print('\n[Ctrl+C] 종료')
    finally:
        if max_decel < 0:
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
