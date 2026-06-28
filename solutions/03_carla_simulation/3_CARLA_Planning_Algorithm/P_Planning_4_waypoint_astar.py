#!/usr/bin/env python3
#======================================================================#
# Module:      P_Planning_4_waypoint_astar.py
# Description: A* path planning on the CARLA road network with path following
#
# Authors: Jisang Yun (jisangyun@hanyang.ac.kr)
#
# Revision History
#      June 28, 2026: Jisang Yun - Created.
#======================================================================#
import heapq
import math
import time
import carla

import matplotlib
import matplotlib.pyplot as plt

# ============================================================
# 설정값 (여기만 바꾸면 동작이 달라집니다)
# ============================================================
# 맵: None = 현재 시뮬레이터에 로드된 맵 그대로 사용
#     'TownXX' = 해당 맵을 새로 로드
MAP_NAME      = None    # 예) 'Town01','Town02','Town03','Town04','Town05','Town10HD_Opt'
RESOLUTION    = 2.0     # 도로망 샘플링 간격 = 그리드 해상도 (m)
FIXED_DELTA_S = 0.1     # 시뮬레이션 한 tick 의 시간 (s)

# 주행(Pure Pursuit) 파라미터
GOAL_REACH_M = 3.0      # 목적지 도달 판정 반경 (m)
LOOKAHEAD_M  = 6.0      # 전방 목표점 거리 (m). 크면 부드럽고 작으면 민첩
TARGET_SPEED = 7.0      # 목표 속도 (m/s, 약 25 km/h)

# ============================================================
# 공용 유틸
# ============================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def get_speed_mps(vehicle):
    v = vehicle.get_velocity()
    return math.hypot(v.x, v.y)

# 도로 위 위치를 격자 칸 (gx, gy) 로 양자화해 그래프 노드 키로 사용한다.
# (서로 다른 waypoint.id 라도 같은 물리 위치면 같은 노드로 합쳐지게 만들기 위함)
def node_key(location):
    return (round(location.x / RESOLUTION), round(location.y / RESOLUTION))

def key_to_xy(key):
    return (key[0] * RESOLUTION, key[1] * RESOLUTION)

# ============================================================
# A* 알고리즘 (Day1 복습)
# ============================================================
def heuristic(key_a, key_b):
    #-[TODO]- Return the straight-line distance between two grid nodes
    #[key functions] key_to_xy(key) -> (x, y),  math.hypot(dx, dy)
    ax, ay = key_to_xy(key_a)
    bx, by = key_to_xy(key_b)
    return math.hypot(ax - bx, ay - by)

def astar(graph, start, goal):
    #-[TODO]- Find the shortest path from start to goal with A*, return the node-key list in path order
    #  [algorithm]
    #   1) push (f=0.0, start) into the priority queue open_set
    #   2) start with g_score = {start: 0.0}, came_from = {}
    #   3) pop the node (current) with the smallest f from open_set
    #   4) if current is goal, rebuild the path by following came_from backward and return it
    #   5) else for each neighbor: tentative_g = g_score[current] + edge_cost
    #        if smaller than before, update came_from/g_score and push with f = g + heuristic(neighbor, goal)
    #   6) return [] (no path) when the queue empties
    #[key functions] heapq.heappush(heap, (f, node)), heapq.heappop(heap), graph.get(node, [])
    open_set  = [(0.0, start)]
    came_from = {}
    g_score   = {start: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))

        for neighbor, edge_cost in graph.get(current, []):
            tentative_g = g_score[current] + edge_cost
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor]   = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))
    return []   # 경로 없음

# ============================================================
# 도로망 그래프 빌드 (위상정보 기반 — 교차로 연결 보장)
#   get_topology() 는 (구간 시작 wp, 구간 끝 wp) 쌍 목록을 준다.
#   각 구간을 wp.next(RESOLUTION) 으로 잘게 쪼개 그리드 노드로 연결한다.
# ============================================================
def build_road_graph(world, resolution):
    print('[GRAPH] 도로망 위상정보로 그래프 구성 중...')
    topology = world.get_map().get_topology()

    graph   = {}   # node_key -> [(neighbor_key, cost), ...]
    node_xy = {}   # node_key -> (x, y)

    def add_node(wp):
        k = node_key(wp.transform.location)
        if k not in graph:
            graph[k]   = []
            node_xy[k] = (wp.transform.location.x, wp.transform.location.y)
        return k

    def add_edge(a, b):
        if a != b:
            graph[a].append((b, heuristic(a, b)))

    for seg_begin, seg_end in topology:
        prev_k = add_node(seg_begin)
        wp     = seg_begin
        for _ in range(500):   # 무한 루프 방지 상한
            if wp.transform.location.distance(seg_end.transform.location) <= resolution:
                break
            #-[TODO]- Get the next waypoint candidates resolution(m) ahead of wp
            #[key functions] wp.next(distance) -> list[Waypoint]
            nxts = wp.next(resolution)
            if not nxts:
                break
            wp = nxts[0]
            k  = add_node(wp)
            add_edge(prev_k, k)   # 직전 노드 → 현재 노드 연결
            prev_k = k
        end_k = add_node(seg_end)
        add_edge(prev_k, end_k)

    edges = sum(len(v) for v in graph.values())
    print(f'[GRAPH] 노드: {len(graph)}, 엣지: {edges}')
    return graph, node_xy

def nearest_node(node_xy, location):
    best, best_d = None, float('inf')
    for k, (x, y) in node_xy.items():
        d = (x - location.x) ** 2 + (y - location.y) ** 2
        if d < best_d:
            best_d, best = d, k
    return best

# ============================================================
# Spectator chase 카메라 (NGV_P_Basic_8 방식)
#   차 뒤쪽(-8m)·위(3m)에서 비스듬히(pitch -12) 따라본다.
# ============================================================
def spectator_follow_tf(ego_transform):
    offset = carla.Location(x=-8.0, y=0.0, z=3.0)
    cam_location = ego_transform.transform(offset)   # 차 로컬 → 월드 좌표
    cam_rotation = carla.Rotation(roll=0.0, pitch=-12.0, yaw=ego_transform.rotation.yaw)
    return carla.Transform(cam_location, cam_rotation)

# ============================================================
# Pure Pursuit 경로 추종 제어
#   전방 lookahead 목표점을 향해 조향하고, 목표 속도를 유지한다.
#   (계획된 경로를 '직접' 추종하는 planning → control 연결 부분)
# ============================================================
def pure_pursuit_control(ego_vehicle, target_loc, target_speed):
    tf    = ego_vehicle.get_transform()
    loc   = tf.location
    fwd   = tf.get_forward_vector()
    right = tf.get_right_vector()

    # 목표점을 차량 로컬 좌표의 전방/우측 성분으로 분해
    dx, dy = target_loc.x - loc.x, target_loc.y - loc.y
    local_ahead = dx * fwd.x + dy * fwd.y      # 전방(+) 성분
    local_side  = dx * right.x + dy * right.y  # 우측(+) 성분

    #-[TODO]- Steer toward the target: positive(right) if the target is on the right, negative(left) if on the left
    #         steer = (right component) / max(|forward component|, 1.0), clamped to [-1, 1]
    #[key functions] clamp(v, lo, hi)
    steer = clamp(local_side / max(abs(local_ahead), 1.0), -1.0, 1.0)

    speed    = get_speed_mps(ego_vehicle)
    throttle = 0.6 if speed < target_speed else 0.0
    brake    = 0.0
    if local_ahead < 0:        # 목표가 뒤쪽이면 감속
        throttle, brake = 0.0, 0.3
    return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)

# ============================================================
# matplotlib 시각화 + 클릭으로 출발/도착 선택
# ============================================================
def pick_start_goal(node_xy, spawn_points):
    matplotlib.rcParams['font.size'] = 9
    plt.ion()   # 인터랙티브 모드: 주행 중 실시간 갱신 가능
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.set_title('Road Network - click 2 points: START then GOAL\n'
                 '(snaps to nearest spawn point)')

    # 도로망 (회색 점)
    xs = [xy[0] for xy in node_xy.values()]
    ys = [xy[1] for xy in node_xy.values()]
    ax.scatter(xs, ys, s=1, c='lightgray', label='road network')

    # spawn point (파란 점 + 번호)
    sx = [sp.location.x for sp in spawn_points]
    sy = [sp.location.y for sp in spawn_points]
    ax.scatter(sx, sy, s=18, c='royalblue', zorder=3, label='spawn points')
    for i, sp in enumerate(spawn_points):
        ax.annotate(str(i), (sp.location.x, sp.location.y), fontsize=6, color='navy')

    ax.set_aspect('equal')
    ax.invert_yaxis()   # CARLA top-down 방향과 맞춤
    ax.legend(loc='upper right')
    plt.tight_layout()

    print('[INFO] 창에서 출발/도착 지점을 순서대로 클릭하세요 (2번).')
    pts = plt.ginput(2, timeout=0)   # 2번 클릭까지 대기
    if len(pts) < 2:
        plt.close(fig)
        return None

    def nearest_spawn(px, py):
        best, best_d = 0, float('inf')
        for i, sp in enumerate(spawn_points):
            d = (sp.location.x - px) ** 2 + (sp.location.y - py) ** 2
            if d < best_d:
                best_d, best = d, i
        return best

    start_idx = nearest_spawn(*pts[0])
    goal_idx  = nearest_spawn(*pts[1])
    print(f'[INFO] 선택됨 → 출발: spawn[{start_idx}], 도착: spawn[{goal_idx}]')

    for idx, color, name in [(start_idx, 'lime', 'START'), (goal_idx, 'red', 'GOAL')]:
        sp = spawn_points[idx]
        ax.scatter([sp.location.x], [sp.location.y], s=120, c=color,
                   edgecolors='black', zorder=5)
        ax.annotate(name, (sp.location.x, sp.location.y),
                    fontsize=11, fontweight='bold', color=color)
    plt.draw()
    return fig, ax, start_idx, goal_idx

def draw_route(ax, node_xy, path_nodes):
    px = [node_xy[k][0] for k in path_nodes]
    py = [node_xy[k][1] for k in path_nodes]
    ax.plot(px, py, '-', color='orange', linewidth=3, zorder=4, label='A* path')
    ax.legend(loc='upper right')
    plt.draw()
    plt.pause(0.1)

# ============================================================
# Main
# ============================================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)

    # ============================================================
    # 맵 로드 (설정에 따라)
    # ============================================================
    if MAP_NAME:
        print(f'[INFO] 맵 로드: {MAP_NAME}')
        world = client.load_world(MAP_NAME)
    else:
        world = client.get_world()

    original_settings = world.get_settings()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_S
    world.apply_settings(settings)

    actor_list = []
    fig = None

    try:
        spawn_points = world.get_map().get_spawn_points()

        # ============================================================
        # 도로망 그래프 빌드
        # ============================================================
        graph, node_xy = build_road_graph(world, RESOLUTION)

        # ============================================================
        # 창에서 출발/도착 클릭 선택
        # ============================================================
        picked = pick_start_goal(node_xy, spawn_points)
        if picked is None:
            print('[INFO] 선택이 취소되었습니다.')
            return
        fig, ax, start_idx, goal_idx = picked
        start_tf = spawn_points[start_idx]
        goal_tf  = spawn_points[goal_idx]

        # ============================================================
        # A* 경로 탐색
        # ============================================================
        start_node = nearest_node(node_xy, start_tf.location)
        goal_node  = nearest_node(node_xy, goal_tf.location)
        print('[A*] 탐색 시작...')
        # 위에서 구현한 A* 로 경로 탐색
        path_nodes = astar(graph, start_node, goal_node)

        if not path_nodes:
            print('[A*] 경로를 찾지 못했습니다. 다른 도착지를 선택해보세요.')
            print('     (일방통행/분리된 도로라 전진만으로 도달 불가일 수 있습니다)')
            plt.show()
            return

        # 경로 노드 → 실제 좌표(carla.Location) 리스트
        path_locs = [carla.Location(x=node_xy[k][0], y=node_xy[k][1], z=start_tf.location.z)
                     for k in path_nodes]
        total_len = sum(path_locs[i].distance(path_locs[i+1])
                        for i in range(len(path_locs)-1))
        print(f'[A*] 경로 탐색 완료: {len(path_nodes)} nodes, 약 {total_len:.0f} m')

        # ============================================================
        # 창 + CARLA 에 경로 표시
        # ============================================================
        draw_route(ax, node_xy, path_nodes)
        for loc in path_locs[::3]:
            world.debug.draw_string(
                carla.Location(loc.x, loc.y, loc.z + 0.5), 'O',
                draw_shadow=False, color=carla.Color(0, 200, 0), life_time=120.0)

        # ============================================================
        # 차량 Spawn
        # ============================================================
        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name', 'ego')
        ego_bp.set_attribute('color', '255,0,0')
        ego_vehicle = world.try_spawn_actor(ego_bp, start_tf)
        if ego_vehicle is None:
            raise RuntimeError('ego spawn 실패')
        actor_list.append(ego_vehicle)
        world.tick()

        # ============================================================
        # 경로 추종 주행 (Pure Pursuit)
        # ============================================================
        ego_vehicle.set_autopilot(False)   # TM 대신 우리가 직접 제어
        spectator  = world.get_spectator()
        goal_loc   = path_locs[-1]
        print('[INFO] 차량이 A* 경로를 따라 주행합니다. Ctrl+C 로 종료.')

        target_idx = 0
        step = 0
        reached = False
        while not reached:
            tf  = ego_vehicle.get_transform()
            loc = tf.location

            # 전방 lookahead 지점 선택 (가까워진 노드는 건너뜀)
            while target_idx < len(path_locs) - 1 and \
                    loc.distance(path_locs[target_idx]) < LOOKAHEAD_M:
                target_idx += 1
            target_loc = path_locs[target_idx]

            # 목적지 도달 판정 → 정지
            if loc.distance(goal_loc) <= GOAL_REACH_M:
                ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                print(f'[GOAL] 목적지 도착 — 차량 정지. (거리 {loc.distance(goal_loc):.1f}m)')
                reached = True
            else:
                ego_vehicle.apply_control(pure_pursuit_control(ego_vehicle, target_loc, TARGET_SPEED))

            spectator.set_transform(spectator_follow_tf(tf))   # chase 카메라

            # matplotlib 차 위치 갱신 (10 tick 마다, 블로킹 없는 draw)
            step += 1
            if step % 10 == 0:
                ax.scatter([loc.x], [loc.y], s=8, c='red', zorder=6)
                fig.canvas.draw_idle()
                fig.canvas.flush_events()

            world.tick()
            time.sleep(FIXED_DELTA_S)   # 일정한 실시간 페이싱 (끊김 방지)

        # 도착 표시 + 정지 상태 유지 (창 닫거나 Ctrl+C 까지)
        ax.scatter([goal_loc.x], [goal_loc.y], s=120, c='red',
                   marker='*', edgecolors='black', zorder=7)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        print('[INFO] 창을 닫거나 Ctrl+C 로 종료하세요.')
        while True:
            ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
            world.tick()
            time.sleep(FIXED_DELTA_S)

    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        client.apply_batch([carla.command.DestroyActor(a.id) for a in actor_list])
        world.apply_settings(original_settings)
        plt.close('all')
        print('[DONE] 정리 완료')


if __name__ == '__main__':
    main()
