#!/usr/bin/env python3
#======================================================================#
# Module:      P_Planning_2_ttc_decision.py
# Description: Compute Time-To-Collision (TTC) and brake the ego_vehicle accordingly
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
TOWN              = 'Town04'
FIXED_DELTA_S     = 0.05
EGO_SPAWN_INDEX   = 0
LEAD_FORWARD_M    = 30.0    # Ego 전방 선행차 초기 거리
EGO_TARGET_KMH   = 55.0
LEAD_TARGET_KMH  = 45.0
LEAD_DECEL_KMH   = 5.0     # 선행차 감속 목표 속도
DECEL_TRIGGER_S  = 3.0     # 시뮬레이션 시작 후 몇 초 후에 선행차 감속 시작
LEAD_STOP_AFTER_S = 4.0     # 감속 시작 후 몇 초 뒤 정지
LEAD_RESUME_AFTER_S = 8.0   # 감속 시작 후 몇 초 뒤 다시 출발
MAX_TICKS         = 600
LOG_PATH          = Path('output/ttc_log.csv')

# TTC 임계값
TTC_EMERGENCY = 1.5   # (s) 이하 → 비상 제동
TTC_BRAKE     = 3.0   # (s) 이하 → 제동
TTC_CAUTION   = 5.0   # (s) 이하 → 주의 (차선 변경 검토)
TTC_RESUME    = 6.0   # (s) 이상일 때만 제동 상태를 풀어보기
SAFE_GAP_M    = 15.0  # (m) 제동 해제용 최소 간격

# ============================================================
# TTC 계산
# ============================================================
def compute_ttc(gap_m, rel_speed_mps):
    #-[TODO]- Implement TTC = gap / rel_speed (return inf if rel_speed <= 0)
    #         rel_speed = ego speed - lead speed (positive means closing in)
    #[key functions] simple division, boundary check
    if rel_speed_mps <= 1e-3:
        return float('inf')
    return gap_m / rel_speed_mps

# ============================================================
# TTC 기반 판단
# ============================================================
def decide(ttc, gap_m):
    #-[TODO]- Decide the action from TTC: EMERGENCY_BRAKE / BRAKE / CAUTION / KEEP
    #[key functions] compare ttc with TTC_EMERGENCY / TTC_BRAKE / TTC_CAUTION
    if ttc < TTC_EMERGENCY:
        return 'EMERGENCY_BRAKE'
    if ttc < TTC_BRAKE:
        return 'BRAKE'
    if ttc < TTC_CAUTION:
        return 'CAUTION'
    return 'KEEP'

# ============================================================
# Util
# ============================================================
def get_speed_mps(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x**2 + v.y**2 + v.z**2)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def set_tm_speed(traffic_manager, vehicle, target_kmh):
    limit = max(1.0, float(vehicle.get_speed_limit()))
    pct   = ((limit - float(target_kmh)) / limit) * 100.0
    traffic_manager.vehicle_percentage_speed_difference(vehicle, clamp(pct, -80.0, 95.0))

def set_initial_velocity(vehicle, kmh):
    fwd = vehicle.get_transform().get_forward_vector()
    mps = kmh / 3.6
    vehicle.set_target_velocity(carla.Vector3D(fwd.x * mps, fwd.y * mps, 0.0))

# ============================================================
# Main
# ============================================================
def main():
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
    tm_port = traffic_manager.get_port()

    actor_list = []
    history    = []
    collision  = {'hit': False, 'other': None}
    ego_autopilot = True
    brake_hold = False
    decel_trigger_tick = int(DECEL_TRIGGER_S / FIXED_DELTA_S)
    lead_stop_tick = int((DECEL_TRIGGER_S + LEAD_STOP_AFTER_S) / FIXED_DELTA_S)
    lead_resume_tick = int((DECEL_TRIGGER_S + LEAD_RESUME_AFTER_S) / FIXED_DELTA_S)

    try:
        for v in world.get_actors().filter('vehicle.*'):
            v.destroy()
        world.tick()

        sps = world.get_map().get_spawn_points()

        # ============================================================
        # Ego Spawn
        # ============================================================
        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('color', '255,0,0')
        sp  = sps[EGO_SPAWN_INDEX]
        ego_vehicle = world.try_spawn_actor(ego_bp, carla.Transform(
            carla.Location(sp.location.x, sp.location.y, sp.location.z + 0.3),
            sp.rotation))
        if ego_vehicle is None:
            raise RuntimeError('ego spawn 실패')
        actor_list.append(ego_vehicle)
        world.tick()

        # ============================================================
        # 선행차 Spawn (ego_vehicle 전방 같은 차선)
        # ============================================================
        ego_wp = world.get_map().get_waypoint(
            ego_vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)

        #-[TODO]- Get the lead vehicle's spawn waypoint LEAD_FORWARD_M ahead of the ego
        #[key functions] wp.next(distance) -> list[Waypoint]
        lead_wps = ego_wp.next(LEAD_FORWARD_M)
        if not lead_wps:
            raise RuntimeError('선행차 spawn 위치 없음')
        lead_tf = lead_wps[0].transform

        lead_bp = world.get_blueprint_library().find('vehicle.audi.tt')
        lead_vehicle    = world.try_spawn_actor(lead_bp, carla.Transform(
            carla.Location(lead_tf.location.x, lead_tf.location.y, lead_tf.location.z + 0.3),
            lead_tf.rotation))
        if lead_vehicle is None:
            raise RuntimeError('선행차 spawn 실패')
        actor_list.append(lead_vehicle)
        world.tick()

        # ============================================================
        # 충돌 센서
        # ============================================================
        col_bp = world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor    = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego_vehicle)
        collision_sensor.listen(lambda e: collision.update(hit=True, other=e.other_actor.type_id))
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
        for v in (ego_vehicle, lead_vehicle):
            traffic_manager.ignore_lights_percentage(v, 100.0)
        set_tm_speed(traffic_manager, ego_vehicle, EGO_TARGET_KMH)
        set_tm_speed(traffic_manager, lead_vehicle, LEAD_TARGET_KMH)

        print(f'[INFO] TTC Decision 시나리오 시작 ({DECEL_TRIGGER_S}s 후 선행차 감속)')
        print(f'       TTC 임계값: EMERGENCY<{TTC_EMERGENCY}s  BRAKE<{TTC_BRAKE}s  CAUTION<{TTC_CAUTION}s')
        spectator = world.get_spectator()

        for tick in range(MAX_TICKS):
            world.tick()
            time.sleep(FIXED_DELTA_S)   # 실시간 페이싱 (너무 빨리 진행되지 않게)
            elapsed = tick * FIXED_DELTA_S

            if collision['hit']:
                print(f'[FAIL] 충돌 발생: {collision["other"]}')
                break

            # spectator 업데이트 (ego_vehicle 뒤 chase view)
            tf  = ego_vehicle.get_transform()
            fwd = tf.get_forward_vector()
            spectator.set_transform(carla.Transform(
                carla.Location(tf.location.x - fwd.x*8, tf.location.y - fwd.y*8, tf.location.z + 5),
                carla.Rotation(pitch=-15, yaw=tf.rotation.yaw)))

            # 선행차 감속 (트리거 시점 이후)
            if tick >= lead_resume_tick:
                set_tm_speed(traffic_manager, lead_vehicle, LEAD_TARGET_KMH)
            elif tick >= lead_stop_tick:
                set_tm_speed(traffic_manager, lead_vehicle, 0.0)
            elif tick >= decel_trigger_tick:
                set_tm_speed(traffic_manager, lead_vehicle, LEAD_DECEL_KMH)

            # TTC 계산
            gap      = max(ego_vehicle.get_location().distance(lead_vehicle.get_location()) - 4.5, 0.0)
            ego_mps  = get_speed_mps(ego_vehicle)
            lead_mps = get_speed_mps(lead_vehicle)

            # 위에서 구현한 함수로 TTC 계산 → 행동 결정
            ttc    = compute_ttc(gap, ego_mps - lead_mps)
            action = decide(ttc, gap)

            # 판단 결과 → Ego 제어
            if action in ('BRAKE', 'EMERGENCY_BRAKE'):
                if ego_autopilot:
                    ego_vehicle.set_autopilot(False, tm_port)
                    ego_autopilot = False
                brake_hold = True
                brake_val = 1.0 if action == 'EMERGENCY_BRAKE' else 0.5
                #-[TODO]- Apply the brake command with ego_vehicle.apply_control()
                #[key functions] carla.VehicleControl(throttle=0.0, brake=brake_val)
                ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=brake_val))
            else:
                if brake_hold:
                    if gap > SAFE_GAP_M and ttc > TTC_RESUME:
                        brake_hold = False
                        ego_vehicle.set_autopilot(True, tm_port)
                        ego_autopilot = True
                    else:
                        ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.5))
                elif not ego_autopilot:
                    ego_vehicle.set_autopilot(True, tm_port)
                    ego_autopilot = True

            # 로그 기록 및 출력
            ttc_display = min(ttc, 10.0)
            history.append((round(elapsed, 2), round(gap, 2), round(ttc_display, 2), action))

            if tick % 20 == 0:
                print(f'  t={elapsed:.1f}s  gap={gap:.1f}m  TTC={ttc_display:.2f}s  → {action}')

        else:
            print('[SUCCESS] 시나리오 종료 (충돌 없음)')

    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        # 로그 저장
        if history:
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with LOG_PATH.open('w', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                w.writerow(['time_s', 'gap_m', 'ttc_s', 'action'])
                w.writerows(history)
            print(f'[INFO] 로그 저장: {LOG_PATH}')

        client.apply_batch([carla.command.DestroyActor(a.id) for a in actor_list])
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print('[DONE] 정리 완료')


if __name__ == '__main__':
    main()
