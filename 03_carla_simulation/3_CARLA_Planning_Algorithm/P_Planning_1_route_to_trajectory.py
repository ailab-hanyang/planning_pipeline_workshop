#!/usr/bin/env python3
#======================================================================#
# Module:      P_Planning_1_route_to_trajectory.py
# Description: 경로 -> 궤적 -> 주행 (Day 1 A* + Day 2 Spline·속도 계획을 CARLA 도로에서 재현)
#              도로망 A* -> cubic spline -> 곡률 제한 속도 + 가·감속 smoothing -> 경로 추종
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# Revision History
#      June 28, 2026: Jisang Yun - Created. (P_Planning_4_waypoint_astar)
#      Jul  04, 2026: Jisang Yun - A* 설명 보강
#      Oct  19, 2026: Jisang Yun - Planning_1 로 재구성: Day 2 spline·속도 프로파일 연결
#======================================================================#
# 흐름 (자율주행 planning 계층)
#   ① 경로(Global)  : 도로망 그래프 + A*                -> 2 m 간격 점 목록          (Day 1 Step02)
#                     (점을 이으면 커브·차선 변경에서 각짐 -> ② 에서 다듬음)
#   ② 경로 다듬기   : cubic spline                     -> 0.5 m 간격 부드러운 곡선  (Day 2 Step01)
#   ③ 속도 계획     : 곡률 제한 속도 + 가·감속 smoothing -> 커브 전 미리 감속        (Day 2 Step02)
#      ② + ③ = 궤적(Motion): 경로 + 각 지점의 속도
#   ④ 제어(Control) : 경로 추종 (조향 + 목표 속도)
#
# [실험] USE_SPEED_PROFILE = False -> 항상 V_MAX 로 주행 (속도 계획 없음)
#        오른쪽 아래 '횡가속도' 그래프를 A_LAT_MAX(2.0 m/s²) 선과 비교
#   ※ OFF 여도 차는 코너를 돎: A_LAT_MAX 는 '승차감' 한계 (타이어가 미끄러지는 한계는 마른 노면 약 8~9 m/s²,
#      CARLA 차량은 타이어 접지력이 실제보다 커서 더 높음). 비교 포인트 = 코너를 도느냐가 아니라 '얼마나 불편하게 도느냐'
#======================================================================#
import heapq
import math
import os
import sys
import time

import numpy as np
import matplotlib.pyplot as plt
import carla


# Day 2 utils 의 CubicSpline 재사용: 상위 폴더에서 utils/02_trajectory_planning 찾기
def _add_day2_utils():
    folder = os.path.dirname(os.path.abspath(__file__))
    while folder != os.path.dirname(folder):
        candidate = os.path.join(folder, 'utils', '02_trajectory_planning')
        if os.path.isdir(candidate):
            sys.path.insert(0, candidate)
            return
        folder = os.path.dirname(folder)
    raise RuntimeError('utils/02_trajectory_planning 폴더를 찾지 못함 (저장소 구조 그대로 실행)')

_add_day2_utils()
from step01_cubic_spline_path import CalcSplineCourse  # noqa: E402  (Day 2 Step01 spline: x(s), y(s), 곡률 κ)

# ============================================================
# 설정값
# ============================================================
MAP_NAME      = 'Town10HD_Opt'
START_IDX     = 128      # 출발 spawn point 번호 (교차로 우회전 포함, 약 275 m)
GOAL_IDX      = 108      # 도착 spawn point 번호
PICK_BY_CLICK = False    # True -> 지도 창에서 출발/도착 클릭으로 선택

RESOLUTION    = 2.0      # 도로망 샘플링 간격 (m)
LANE_CHANGE_M       = 8.0   # 차선 변경 엣지: 옆 차선의 몇 m 앞 노드로 연결할지
LANE_CHANGE_PENALTY = 5.0   # 차선 변경 비용 가산 (m 환산). 불필요한 차선 변경 억제
FIXED_DELTA_S = 0.05     # 시뮬레이션 한 tick 의 시간 (s) -> 화면 20 프레임/s

# 속도 계획 파라미터 (Day 2 Step02 와 같은 의미)
USE_SPEED_PROFILE = True
V_MAX     = 7.0          # 최고 속도 (m/s, 약 25 km/h) -- 도심 제한속도 역할
A_LAT_MAX = 2.0          # 횡가속도 한계 (m/s², 승차감 기준) -> 곡률 제한 속도
A_MAX     = 1.5          # 종방향 가속 한계 (m/s²)
A_MIN     = -2.5         # 종방향 감속 한계 (m/s², 음수)
DS        = 0.5          # spline 샘플 간격 (m)

# 경로 추종 파라미터
GOAL_REACH_M = 3.0       # 목적지 도달 판정 반경 (m)
LOOKAHEAD_M  = 6.0       # 전방 목표점 거리 (m)

# ============================================================
# 공용 유틸
# ============================================================
def clamp(value, low, high):
    return max(low, min(high, value))

def get_speed_mps(vehicle):
    velocity = vehicle.get_velocity()
    return math.hypot(velocity.x, velocity.y)

# 그래프 노드 -> 실제 좌표 (x, y). build_road_graph() 가 채우고 heuristic() 이 읽음
NODE_XY = {}

# 차선 구간의 양 끝(연결점)은 위치로 식별: 1 m 단위 반올림이 같으면 같은 연결점
# (앞 구간의 끝 = 다음 구간의 시작 -> 이렇게 합쳐야 도로가 이어짐. z 포함 -> 고가/지하 구분)
def endpoint_key(location):
    return ('pt', round(location.x), round(location.y), round(location.z))

# ============================================================
# ① 경로: A* (Day 1 Step02 복습)
# ------------------------------------------------------------
#   f = g + h 가 가장 작은 노드부터 확장
#     g = 출발 -> 현재 노드까지 실제 비용   (g_score)
#     h = 현재 노드 -> 목표까지 추정 비용   (heuristic, 직선거리)
#   Day 1 대응: Open Set = open_set (heap) / Closed Set = visited / parent_index = came_from
# ============================================================
def heuristic(key_a, key_b):
    # h(추정치): 두 점 사이 직선(유클리드) 거리
    # 직선거리: 실제 도로거리보다 길 수 없음(admissible), 엣지 하나 이동 시 엣지 길이보다 더 줄지 않음(consistent) -> 최단 경로 보장
    a_x, a_y = NODE_XY[key_a]
    b_x, b_y = NODE_XY[key_b]
    ##### TODO 1 #####
    # 두 점 (a_x, a_y) 와 (b_x, b_y) 사이의 직선(유클리드) 거리 구하기
    #   - dx = a_x - b_x, dy = a_y - b_y   (아직 없는 변수 -> 직접 계산해 사용)
    # 사용함수: math.hypot(dx, dy) / 기능: sqrt(dx*dx + dy*dy), 즉 두 점 사이 직선 거리
    # 주의: math.hypot 은 인자 개수 제한 없음 -> hypot(a_x, a_y, b_x, b_y) 처럼 4개 넣어도 에러 없이 오답
    # 확인: main() 시작 시 (0,0)->(3,4) = 5.0 자동 검사
    #################

    return math.hypot(...)

    #################

def astar(graph, start, goal):
    # A* 로 start -> goal 최단 경로 탐색. (경로 노드 리스트, 확장한 노드 집합) 반환
    open_set  = [(0.0, start)]   # (f, node) 우선순위 큐 (Day 1 Open Set)
    came_from = {}               # 경로 복원용: neighbor 는 current 를 거쳐서 옴 (Day 1 parent_index)
    g_score   = {start: 0.0}     # 출발 -> 각 노드까지 실제 최소 비용 (g)
    visited   = set()            # 확장(expand) 완료 노드 (Day 1 Closed Set)

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:      # 도착 -> came_from 을 거꾸로 따라 경로 복원
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path)), visited
        if current in visited:   # 이미 확장한 노드면 건너뜀 (중복 push 대비)
            continue
        visited.add(current)

        for neighbor, edge_cost in graph.get(current, []):
            tentative_g = g_score[current] + edge_cost       # current 를 거쳐 neighbor 로 가는 비용
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor]   = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)   # f = g + h
                heapq.heappush(open_set, (f_score, neighbor))
    return [], visited   # 경로 없음

# ============================================================
# 도로망 그래프 빌드 (위상정보 기반 -- 교차로 연결 보장)
#   get_topology() = (구간 시작 waypoint, 구간 끝 waypoint) 쌍 목록
#   - 각 구간을 waypoint.next(RESOLUTION) 으로 잘게 쪼개 노드로 연결
#   - 구간 양 끝(연결점)만 위치로 합치고, 구간 내부 점은 구간마다 별도 노드
#     -> 교차로 안에서 서로 교차하는 차선(직진 vs 좌회전)끼리 섞이지 않음
#   - 교차로 밖 '차선 변경 허용' 구간에만 옆 차선으로 가는 엣지 추가
# ============================================================
def build_road_graph(world, resolution):
    print('[GRAPH] 도로망 위상정보로 그래프 구성 중...')
    topology = world.get_map().get_topology()

    graph = {}         # node -> [(neighbor, cost), ...]
    NODE_XY.clear()    # node -> (x, y)
    node_wp = {}       # 구간 내부 노드 -> waypoint (차선 변경 엣지용)
    lane_nodes = {}    # (road_id, section_id, lane_id) -> [(s, node), ...]

    def add_node(node, waypoint):
        if node not in graph:
            graph[node]   = []
            NODE_XY[node] = (waypoint.transform.location.x, waypoint.transform.location.y)
        return node

    def add_edge(from_key, to_key):
        # 엣지 비용 = 인접 노드 사이 실제 거리 (heuristic 과 같은 수식, 다른 역할)
        if from_key != to_key:
            graph[from_key].append((to_key, heuristic(from_key, to_key)))

    for seg_idx, (seg_begin, seg_end) in enumerate(topology):
        prev_node = add_node(endpoint_key(seg_begin.transform.location), seg_begin)
        waypoint  = seg_begin
        for step in range(500):   # 무한 루프 방지 상한 (2 m x 500 = 1 km)
            if waypoint.transform.location.distance(seg_end.transform.location) <= resolution:
                break
            next_waypoints = waypoint.next(resolution)   # 차선을 따라 resolution(m) 앞의 waypoint
            if not next_waypoints:
                break
            waypoint = next_waypoints[0]
            node = add_node(('seg', seg_idx, step), waypoint)   # 구간 내부 점: 이 구간 전용 노드
            node_wp[node] = waypoint
            lane_nodes.setdefault((waypoint.road_id, waypoint.section_id, waypoint.lane_id), []).append((waypoint.s, node))
            add_edge(prev_node, node)
            prev_node = node
        end_node = add_node(endpoint_key(seg_end.transform.location), seg_end)
        add_edge(prev_node, end_node)

    # 차선 변경 엣지: 같은 방향 옆 차선의 LANE_CHANGE_M 앞 노드로 비스듬히 연결
    for node, waypoint in node_wp.items():
        if waypoint.is_junction:
            continue
        for allowed, side_waypoint in ((carla.LaneChange.Left, waypoint.get_left_lane()),
                                       (carla.LaneChange.Right, waypoint.get_right_lane())):
            if waypoint.lane_change not in (allowed, carla.LaneChange.Both):
                continue   # 실선 등 차선 변경 금지
            if side_waypoint is None or side_waypoint.lane_type != carla.LaneType.Driving \
                    or side_waypoint.lane_id * waypoint.lane_id < 0:
                continue   # 옆이 주행 차선이 아니거나 반대 방향
            ahead = side_waypoint.next(LANE_CHANGE_M)
            if not ahead:
                continue
            target = ahead[0]
            candidates = lane_nodes.get((target.road_id, target.section_id, target.lane_id))
            if not candidates:
                continue
            target_s, target_node = min(candidates, key=lambda item: abs(item[0] - target.s))
            if abs(target_s - target.s) <= resolution:
                graph[node].append((target_node, heuristic(node, target_node) + LANE_CHANGE_PENALTY))

    edge_count = sum(len(neighbors) for neighbors in graph.values())
    print(f'[GRAPH] 노드: {len(graph)}, 엣지: {edge_count}')
    return graph, NODE_XY

def nearest_node(node_xy, location):
    best_node, best_dist_sq = None, float('inf')
    for node, (x, y) in node_xy.items():
        dist_sq = (x - location.x) ** 2 + (y - location.y) ** 2
        if dist_sq < best_dist_sq:
            best_dist_sq, best_node = dist_sq, node
    return best_node

# ============================================================
# ② 경로 다듬기: A* 의 2 m 간격 점들 -> cubic spline (Day 2 Step01)
#   CalcSplineCourse(x 목록, y 목록, ds) -> (x, y, yaw, κ, s) 를 ds 간격으로 반환
# ============================================================
def smooth_path(path_x, path_y, ds):
    # 너무 가까운 점(0.3 m 미만)은 제외: 같은 위치가 반복되면 spline 계산이 불안정
    keep_x, keep_y = [path_x[0]], [path_y[0]]
    for x, y in zip(path_x[1:], path_y[1:]):
        if math.hypot(x - keep_x[-1], y - keep_y[-1]) >= 0.3:
            keep_x.append(x); keep_y.append(y)
    x, y, yaw, kappa, s = CalcSplineCourse(keep_x, keep_y, ds=ds)
    return np.array(x), np.array(y), np.array(kappa), np.array(s)

# ============================================================
# ③ 속도: 곡률 제한 속도 + 가·감속 smoothing (Day 2 Step02 와 같은 식)
#   출발·도착은 정지 (v = 0)
# ============================================================
def plan_speed_profile(kappa, ds, v_max):
    kappa = np.clip(np.abs(kappa), 1e-4, None)   # 직선(κ≈0) 발산 방지

    ##### TODO 2 #####
    # 곡률 제한 속도 계산하기: a_lat = v²·κ ≤ A_LAT_MAX  ->  v_κ = √(A_LAT_MAX / κ)   (Day 2 §2.2)
    #   - 상한: v_max (거의 직선인 구간)
    # 사용함수: np.sqrt(x) / np.minimum(a, b) / 기능: 원소별 제곱근 / 원소별 최솟값
    #################

    v_kappa = ...

    #################

    v_forward = v_kappa.copy()
    v_forward[0] = 0.0                            # 출발: 정지 상태
    for i in range(1, len(v_forward)):
        ##### TODO 3 #####
        # 가속 한계 반영하기 (Day 2 §3.1): 직전 점 v_forward[i-1] 에서 A_MAX 로 가속해 도달 가능한 값까지
        #   - v_i ≤ √(v_{i-1}² + 2·A_MAX·ds)
        # 사용함수: min(a, b) / math.sqrt(x)
        #################

        v_forward[i] = ...

        #################

    v_final = v_forward.copy()
    v_final[-1] = 0.0                             # 도착: 정지
    for i in range(len(v_final) - 2, -1, -1):
        ##### TODO 4 #####
        # 감속 한계 반영하기 (Day 2 §3.2): 다음 점 v_final[i+1] 까지 A_MIN 으로 줄일 수 있는 값까지
        #   - v_i ≤ √(v_{i+1}² - 2·A_MIN·ds)   (A_MIN < 0 -> 근호 안은 +)
        # 사용함수: min(a, b) / math.sqrt(x)
        #################

        v_final[i] = ...

        #################
    return v_kappa, v_final

# ============================================================
# ④ 제어: 경로 추종 (교육용 단순화)
#   - 조향: 전방 목표점의 '횡방향/전방' 비율 (정식 Pure Pursuit 는 atan(2·L·sinα / Ld))
#   - 속도: 목표 속도보다 느리면 가속, 0.5 m/s 이상 빠르면 약하게 제동
#     (CARLA 차량은 throttle 0 만으로도 약 4.5 m/s² 감속 -> 제동은 거의 필요 없음)
# ============================================================
def follow_control(ego_vehicle, target_location, target_speed):
    ego_transform  = ego_vehicle.get_transform()
    ego_location   = ego_transform.location
    forward_vector = ego_transform.get_forward_vector()
    right_vector   = ego_transform.get_right_vector()

    delta_x, delta_y = target_location.x - ego_location.x, target_location.y - ego_location.y
    local_ahead = delta_x * forward_vector.x + delta_y * forward_vector.y   # 전방(+) 성분
    local_side  = delta_x * right_vector.x   + delta_y * right_vector.y     # 우측(+) 성분
    steer = clamp(local_side / max(abs(local_ahead), 1.0), -1.0, 1.0)       # +: 우회전

    speed    = get_speed_mps(ego_vehicle)
    throttle = 0.6 if speed < target_speed else 0.0
    brake    = 0.3 if speed > target_speed + 0.5 else 0.0
    return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)

def spectator_follow_tf(ego_transform):
    # 차 뒤(-8 m)·위(3 m)에서 비스듬히(pitch -12) 따라보는 chase 카메라
    offset = carla.Location(x=-8.0, y=0.0, z=3.0)
    cam_location = ego_transform.transform(offset)
    cam_rotation = carla.Rotation(pitch=-12.0, yaw=ego_transform.rotation.yaw, roll=0.0)
    return carla.Transform(cam_location, cam_rotation)

# ============================================================
# 시각화: (왼쪽) 지도 + A* 경로 + spline / (오른쪽) 속도 프로파일
# ============================================================
def draw_map(ax, node_xy, spawn_points):
    ax.scatter([p[0] for p in node_xy.values()], [p[1] for p in node_xy.values()],
               s=1, c='lightgray', label='road network')
    ax.scatter([sp.location.x for sp in spawn_points], [sp.location.y for sp in spawn_points],
               s=10, c='royalblue', zorder=3, label='spawn points')
    for i, sp in enumerate(spawn_points):
        ax.annotate(str(i), (sp.location.x, sp.location.y), fontsize=5, color='navy')
    ax.set_aspect('equal')
    ax.invert_yaxis()   # CARLA 위에서 본 방향과 맞춤 (y = 오른쪽)

def pick_start_goal_by_click(ax, spawn_points):
    print('[INFO] 지도 창에서 출발/도착 지점을 순서대로 클릭 (2번)')
    clicked = plt.ginput(2, timeout=0)
    if len(clicked) < 2:
        return None
    def nearest_spawn(px, py):
        return min(range(len(spawn_points)),
                   key=lambda i: (spawn_points[i].location.x - px) ** 2 + (spawn_points[i].location.y - py) ** 2)
    return nearest_spawn(*clicked[0]), nearest_spawn(*clicked[1])

# ============================================================
# Main
# ============================================================
def main():
    # heuristic 자가 확인: (0,0)->(3,4) 직선거리 = 5.0 (CARLA 서버 없이도 확인 가능)
    NODE_XY.update({'check_a': (0.0, 0.0), 'check_b': (3.0, 4.0)})
    assert abs(heuristic('check_a', 'check_b') - 5.0) < 1e-9, \
        'TODO 1 확인 - heuristic 오답: (0,0)->(3,4) 기대값 5.0 -> math.hypot(dx, dy) 인자 2개 확인'
    NODE_XY.clear()

    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    client.load_world_if_different(MAP_NAME)
    world = client.get_world()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_S
    world.apply_settings(settings)

    actor_list = []
    try:
        spawn_points = world.get_map().get_spawn_points()
        graph, node_xy = build_road_graph(world, RESOLUTION)

        plt.rcParams['figure.raise_window'] = False   # 그림 갱신 때마다 창이 맨 앞으로 올라와 다른 창 클릭을 막는 것 방지
        plt.ion()
        fig = plt.figure(figsize=(15, 7.5))
        grid = fig.add_gridspec(2, 2, width_ratios=[1.1, 1])
        ax_map = fig.add_subplot(grid[:, 0])                  # 왼쪽: 지도 + 경로
        ax_v   = fig.add_subplot(grid[0, 1])                  # 오른쪽 위: 속도
        ax_a   = fig.add_subplot(grid[1, 1], sharex=ax_v)     # 오른쪽 아래: 횡가속도
        draw_map(ax_map, node_xy, spawn_points)
        plt.tight_layout(); plt.pause(0.1)

        start_idx, goal_idx = START_IDX, GOAL_IDX
        if PICK_BY_CLICK:
            picked = pick_start_goal_by_click(ax_map, spawn_points)
            if picked is None:
                print('[INFO] 선택 취소'); return
            start_idx, goal_idx = picked
        start_transform = spawn_points[start_idx]
        print(f'[INFO] 출발 spawn[{start_idx}] -> 도착 spawn[{goal_idx}]')

        # ① 경로: A*
        path_nodes, visited = astar(graph, nearest_node(node_xy, start_transform.location),
                                    nearest_node(node_xy, spawn_points[goal_idx].location))
        if not path_nodes:
            print('[A*] 경로 없음 (일방통행 등) -> 다른 도착지 선택'); return
        path_x = [node_xy[n][0] for n in path_nodes]
        path_y = [node_xy[n][1] for n in path_nodes]
        print(f'[A*] 확장 노드 {len(visited)}개, 경로 {len(path_nodes)}개 점')

        # ② 경로 다듬기: spline / ③ 속도 계획: 프로파일 (② + ③ = 궤적)
        xs, ys, kappa, s = smooth_path(path_x, path_y, DS)
        v_kappa, v_plan = plan_speed_profile(kappa, DS, V_MAX)
        if not USE_SPEED_PROFILE:
            v_plan = np.full(len(s), V_MAX)   # 실험: 속도 계획 없이 항상 V_MAX
        print(f'[PLAN] spline {len(s)}개 점, 길이 {s[-1]:.0f} m, 최소 회전반경 {1 / max(np.abs(kappa).max(), 1e-4):.1f} m, '
              f'계획 최저 속도(주행 중) {v_kappa.min() * 3.6:.0f} km/h')

        ax_map.scatter([node_xy[n][0] for n in visited], [node_xy[n][1] for n in visited],
                       s=4, c='skyblue', zorder=2, label='explored (A*)')
        ax_map.plot(path_x, path_y, '.', color='orange', ms=4, zorder=4, label='A* path (2 m points)')
        ax_map.plot(xs, ys, '-', color='green', lw=1.5, zorder=5, label='spline')
        ax_map.legend(loc='upper right', fontsize=8)
        ax_map.set_title(f'Route spawn[{start_idx}] -> spawn[{goal_idx}]')
        ax_v.plot(s, v_kappa * 3.6, '--', color='gray', label='curvature limit v_kappa')
        ax_v.plot(s, v_plan * 3.6, '-', color='tab:blue', lw=2,
                  label='planned speed' if USE_SPEED_PROFILE else 'constant V_MAX (no profile)')
        ax_v.set_ylabel('speed [km/h]')
        ax_v.set_ylim(0, V_MAX * 3.6 * 1.3); ax_v.grid(True, alpha=0.3); ax_v.legend(loc='lower center', fontsize=8)
        ax_v.set_title('Speed profile (red dots = actual)')
        ax_a.axhline(A_LAT_MAX, color='red', ls='--', lw=1.2, label=f'A_LAT_MAX = {A_LAT_MAX} m/s² (comfort)')
        ax_a.set_xlabel('arc length s [m]'); ax_a.set_ylabel('lateral accel [m/s²]')
        ax_a.set_ylim(0, 6.0); ax_a.grid(True, alpha=0.3); ax_a.legend(loc='upper left', fontsize=8)
        ax_a.set_title('Lateral acceleration (actual, 0.5 s average)')
        plt.tight_layout()
        plt.pause(0.1)
        for i in range(0, len(xs), 6):
            world.debug.draw_point(carla.Location(x=float(xs[i]), y=float(ys[i]), z=start_transform.location.z + 0.5),
                                   size=0.08, color=carla.Color(0, 200, 0), life_time=120.0)

        # 차량 spawn
        ego_blueprint = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_blueprint.set_attribute('role_name', 'ego')
        ego_blueprint.set_attribute('color', '255,0,0')
        ego_vehicle = world.try_spawn_actor(ego_blueprint, start_transform)
        if ego_vehicle is None:
            raise RuntimeError('ego spawn 실패 (출발 위치에 다른 차량?)')
        actor_list.append(ego_vehicle)
        world.tick()

        # ④ 제어: 경로 추종
        spectator = world.get_spectator()
        path_locations = [carla.Location(x=float(x), y=float(y), z=start_transform.location.z) for x, y in zip(xs, ys)]
        goal_location  = path_locations[-1]
        target_idx, tick, peak = 0, 0, (0.0, 0.0, 0.0)   # peak = (최대 횡가속도, s, 속도)
        trace_x, trace_y, trace_s, trace_v, trace_a = [], [], [], [], []
        a_lat_raw   = []                         # 순간 횡가속도 (한 tick 값은 튀므로 0.5 s 평균으로 사용)
        avg_window  = int(0.5 / FIXED_DELTA_S)
        trace_line, = ax_map.plot([], [], '-', color='red', lw=1.5, zorder=6)    # 실제 주행 궤적
        speed_dots, = ax_v.plot([], [], '.', color='red', ms=3)                  # 실제 속도
        accel_line, = ax_a.plot([], [], '-', color='tab:purple', lw=1.5)          # 실제 횡가속도
        plot_every  = int(1.0 / FIXED_DELTA_S)   # 그래프는 1초에 한 번만 갱신 (매 tick 그리면 주행이 끊김)
        print('[INFO] 주행 시작. Ctrl+C 로 종료')
        while True:
            loop_start = time.time()
            ego_transform = ego_vehicle.get_transform()
            ego_location  = ego_transform.location
            while target_idx < len(path_locations) - 1 and ego_location.distance(path_locations[target_idx]) < LOOKAHEAD_M:
                target_idx += 1
            if ego_location.distance(goal_location) <= GOAL_REACH_M:
                break
            target_speed = max(float(v_plan[target_idx]), 1.0)   # 전방 목표점의 계획 속도 (정지 직전엔 1 m/s 로 접근)
            ego_vehicle.apply_control(follow_control(ego_vehicle, path_locations[target_idx], target_speed))
            spectator.set_transform(spectator_follow_tf(ego_transform))
            world.tick(); tick += 1

            # 측정: 현재 위치의 s, 속도, 횡가속도 a_lat = v · (yaw 각속도), 0.5 s 평균
            j = int(np.argmin((xs - ego_location.x) ** 2 + (ys - ego_location.y) ** 2))
            speed = get_speed_mps(ego_vehicle)
            a_lat_raw.append(abs(speed * math.radians(ego_vehicle.get_angular_velocity().z)))
            a_lat = sum(a_lat_raw[-avg_window:]) / len(a_lat_raw[-avg_window:])
            if a_lat > peak[0]:
                peak = (a_lat, s[j], speed)
            trace_x.append(ego_location.x); trace_y.append(ego_location.y)
            trace_s.append(s[j]); trace_v.append(speed * 3.6); trace_a.append(a_lat)
            if tick % plot_every == 0:
                trace_line.set_data(trace_x, trace_y)     # 새 점을 추가하지 않고 기존 선의 데이터만 교체 (빠름)
                speed_dots.set_data(trace_s, trace_v)
                accel_line.set_data(trace_s, trace_a)
                fig.canvas.draw_idle(); fig.canvas.flush_events()
                print(f'  s={s[j]:5.0f} m  계획 {v_plan[target_idx] * 3.6:4.0f} km/h  실제 {speed * 3.6:4.0f} km/h  횡가속도 {a_lat:.1f} m/s²')
            time.sleep(max(0.0, FIXED_DELTA_S - (time.time() - loop_start)))   # 실시간 페이싱 (계산 시간 제외)

        ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
        world.tick()
        accel_line.set_data(trace_s, trace_a); speed_dots.set_data(trace_s, trace_v)
        fig.canvas.draw_idle(); fig.canvas.flush_events()
        over_s = np.array(trace_s)[np.array(trace_a) > A_LAT_MAX]          # 한계를 넘은 순간의 위치 s
        over_m = len(np.unique(np.round(over_s / DS))) * DS                  # 한계 초과 구간 길이 (m)
        print(f'[RESULT] 도착: {tick * FIXED_DELTA_S:.0f} s | 최대 횡가속도(0.5 s 평균) {peak[0]:.1f} m/s² '
              f'(s={peak[1]:.0f} m, {peak[2] * 3.6:.0f} km/h) | 승차감 한계 {A_LAT_MAX} m/s² 초과 구간 약 {over_m:.0f} m '
              f'| 속도 계획 {"ON" if USE_SPEED_PROFILE else "OFF"}')
        print('[INFO] 그래프 창을 닫거나 Ctrl+C 로 종료')
        while plt.fignum_exists(fig.number):
            ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
            world.tick()
            fig.canvas.flush_events()   # 창 이벤트만 처리 (plt.pause 는 창을 맨 앞으로 끌어올림)
            time.sleep(FIXED_DELTA_S)

    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        client.apply_batch([carla.command.DestroyActor(actor.id) for actor in actor_list])
        world.apply_settings(original_settings)
        plt.close('all')
        print('[DONE] 정리 완료')


if __name__ == '__main__':
    main()
