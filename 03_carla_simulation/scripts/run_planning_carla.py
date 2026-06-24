#!/usr/bin/env python3
#======================================================================#
# Module:      run_planning_carla.py
# Description: ego-lead 상황에서 TTC 계산 -> Decision(KEEP/BRAKE/EMERGENCY) -> ego 제어
#              (Day1/2 Planning 과 연결되는 상위 판단 계층). 종료 후 그래프/CSV 저장.
#              (carla_template.py 규약 기반)
#
# Authors: AILAB (based on Jisang Yun's carla_template.py)
# CARLA 0.9.16
#======================================================================#
import csv
import math
from pathlib import Path

import carla
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

TOWN = 'Town04'
FIXED_DELTA_S = 0.05
EGO_SPAWN_INDEX = 0
LEAD_FORWARD_M = 30.0
EGO_TARGET_KMH = 55.0
LEAD_TARGET_KMH = 45.0
LEAD_DECEL_KMH = 5.0
DECEL_TRIGGER_TICK = 60
MAX_TICKS = 400
OUT_DIR = Path('output')


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def speed_mps(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def set_tm_target_speed(tm, vehicle, target_kmh):
    speed_limit = max(1.0, float(vehicle.get_speed_limit()))
    pct = ((speed_limit - float(target_kmh)) / speed_limit) * 100.0
    tm.vehicle_percentage_speed_difference(vehicle, clamp(pct, -80.0, 95.0))


def set_initial_velocity(vehicle, kmh):
    fwd = vehicle.get_transform().get_forward_vector()
    mps = kmh / 3.6
    vehicle.set_target_velocity(carla.Vector3D(fwd.x * mps, fwd.y * mps, 0.0))


def compute_ttc(gap, rel_speed):
    if rel_speed <= 1e-3:
        return float('inf')
    return gap / rel_speed


def decide(ttc, gap):
    if ttc < 1.5 or gap < 5:
        return 'EMERGENCY_BRAKE'
    if ttc < 3.0:
        return 'BRAKE'
    if ttc < 5.0:
        return 'LANE_CHANGE?'
    return 'KEEP'


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
    history = []
    collision = {'hit': False, 'other': None}
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
        for v in (ego, lead):
            tm.ignore_lights_percentage(v, 100.0)
            tm.ignore_signs_percentage(v, 100.0)
        set_tm_target_speed(tm, ego, EGO_TARGET_KMH)
        set_tm_target_speed(tm, lead, LEAD_TARGET_KMH)
        print(f'[INFO] {TOWN} TTC decision 시나리오 시작')

        ego_autopilot = True
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

            if tick >= DECEL_TRIGGER_TICK:
                set_tm_target_speed(tm, lead, LEAD_DECEL_KMH)

            gap = max(ego.get_location().distance(lead.get_location()) - 4.5, 0.0)
            ttc = compute_ttc(gap, speed_mps(ego) - speed_mps(lead))
            action = decide(ttc, gap)

            # Decision -> 제어 (실제론 Day1/2 Planning으로 경로 생성)
            if action in ('BRAKE', 'EMERGENCY_BRAKE'):
                if ego_autopilot:
                    ego.set_autopilot(False, tm_port); ego_autopilot = False
                ego.apply_control(carla.VehicleControl(
                    throttle=0.0, brake=1.0 if action == 'EMERGENCY_BRAKE' else 0.5))
            else:
                if not ego_autopilot:
                    ego.set_autopilot(True, tm_port); ego_autopilot = True

            history.append((tick * FIXED_DELTA_S, gap, min(ttc, 10), action))
        else:
            result, reason = 'SUCCESS', 'no collision (timeout)'

    except KeyboardInterrupt:
        result, reason = 'STOPPED', 'keyboard interrupt'
        print('\n[Ctrl+C] 종료')
    finally:
        # 그래프 + CSV 저장
        if history:
            OUT_DIR.mkdir(parents=True, exist_ok=True)
            ts = [h[0] for h in history]
            fig, ax = plt.subplots(2, 1, figsize=(11, 5), sharex=True)
            ax[0].plot(ts, [h[1] for h in history], '-b'); ax[0].set_ylabel('gap [m]'); ax[0].grid(alpha=0.3)
            ax[1].plot(ts, [h[2] for h in history], '-r')
            ax[1].axhline(1.5, ls='--', c='k'); ax[1].axhline(3.0, ls=':', c='gray')
            ax[1].set_ylabel('TTC [s]'); ax[1].set_xlabel('time [s]'); ax[1].grid(alpha=0.3)
            plt.suptitle('Live ego-lead TTC log'); plt.tight_layout()
            fig.savefig(OUT_DIR / 'ttc_log.png', dpi=90, bbox_inches='tight')
            with (OUT_DIR / 'ttc_log.csv').open('w', newline='', encoding='utf-8') as f:
                w = csv.writer(f); w.writerow(['time_s', 'gap_m', 'ttc_s', 'action'])
                w.writerows(history)
            print(f'[INFO] output/ttc_log.png, ttc_log.csv 저장. 마지막: {history[-1]}')
        client.apply_batch([carla.command.DestroyActor(a.id) for a in actor_list])
        tm.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print(f'[RESULT] {result}: {reason}')
        print('[INFO] 정리 완료')


if __name__ == '__main__':
    main()
