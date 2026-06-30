#!/usr/bin/env python3
#======================================================================#
# Module:      P_Planning_1_force_lane_change.py
# Description: Exercise version for Traffic Manager force_lane_change()
#
# Authors: Jisang Yun (jisangyun@hanyang.ac.kr)
#
# Revision History
#      June 28, 2026: Jisang Yun - Created.
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
def has_adjacent_driving_lane(world, spawn_tf, to_right):
    #-[TODO]- Get the waypoint from the spawn location.
    #[key functions] world.get_map().get_waypoint()
    wp = world.get_map().get_waypoint(
        spawn_tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return False

    #-[TODO]- Select the adjacent lane according to the direction flag.
    #[key functions] wp.get_right_lane(), wp.get_left_lane()
    adj = wp.get_right_lane() if to_right else wp.
    if adj is None or adj.lane_type != carla.LaneType.Driving:
        return False

    #-[TODO]- Check that the adjacent lane has the same driving direction.
    #[key functions] lane_id sign check
    return (adj.lane_id < 0) ==


def has_both_adjacent_driving_lanes(world, spawn_tf):
    #-[TODO]- Check whether both left and right adjacent driving lanes exist.
    #[key functions] has_adjacent_driving_lane()
    return has_adjacent_driving_lane(world, spawn_tf, True) and \
        has_adjacent_driving_lane(world, spawn_tf,


def find_spawn_point(world, to_right):
    spawn_points = world.get_map().get_spawn_points()

    #-[TODO]- Find a spawn point that can support lane change in the chosen direction.
    #[key functions] world.get_map().get_spawn_points(), world.get_map().get_waypoint(), wp.get_left_lane(), wp.get_right_lane()
    for spawn_tf in spawn_points:
        if has_both_adjacent_driving_lanes(world, spawn_tf):
            return spawn_tf

    #-[TODO]- Use a fallback spawn point when a fully centered lane is not available.
    for spawn_tf in spawn_points:
        if has_adjacent_driving_lane(world, spawn_tf, to_right):
            return spawn_tf

    return None


def follow_camera_tf(ego_transform):
    # Car 뒤쪽 위에서 따라보는 간단한 spectator 시점
    offset = carla.Location(x=-8.0, y=0.0, z=4.0)
    loc = ego_transform.transform(offset)
    rot = carla.Rotation(roll=0.0, pitch=-15.0, yaw=ego_transform.rotation.yaw)
    return carla.Transform(loc, rot)


def print_vehicle_location(vehicle):
    loc = vehicle.get_location()
    print(f'[POS] x={loc.x:.1f}, y={loc.y:.1f}, z={loc.z:.1f}')


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
    tm_port = traffic_manager.get_port()

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
        spawn_tf = find_spawn_point(world, to_right)
        if spawn_tf is None:
            raise RuntimeError('차선 변경 가능한 spawn point 를 찾지 못함')

        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name', 'ego')

        ego_vehicle = world.try_spawn_actor(ego_bp, spawn_tf)
        if ego_vehicle is None:
            raise RuntimeError('ego spawn 실패')

        #-[TODO]- Enable autopilot for the ego vehicle and connect it to the Traffic Manager.
        #[key functions] ego_vehicle.set_autopilot(True, tm_port), traffic_manager.auto_lane_change(vehicle, True)
        ego_vehicle.set_autopilot(
        traffic_manager.auto_lane_change(ego_vehicle,

        spectator = world.get_spectator()
        start_loc = ego_vehicle.get_location()

        print(f'[START] x={start_loc.x:.1f}, y={start_loc.y:.1f}, z={start_loc.z:.1f}')
        print(f'[INFO] {LANE_CHANGE_DIRECTION.upper()} lane change test start')

        # ============================================================
        # Lane change trigger
        # ============================================================
        #-[TODO]- Force the ego vehicle to change lane after a short delay.
        #[key functions] traffic_manager.force_lane_change(vehicle, to_right: bool)
        # to_right: True = right, False = left
        start_delay_tick = int(START_DELAY_S / FIXED_DELTA_S)
        total_tick = int(RUN_SECONDS / FIXED_DELTA_S)

        for tick in range(total_tick):
            if tick == start_delay_tick:
                traffic_manager.force_lane_change(ego_vehicle,
                print(f'[CMD] force_lane_change({LANE_CHANGE_DIRECTION.upper()})')

            world.tick()
            spectator.set_transform(follow_camera_tf(ego_vehicle.get_transform()))

            #-[TODO]- Print the ego vehicle location so you can check where it is on the map.
            #[key functions] vehicle.get_location()
            if tick % int(1.0 / FIXED_DELTA_S) == 0:
                print_vehicle_location(

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
