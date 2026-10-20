"""
Grid search primitives shared by Dijkstra / A* / Hybrid A*.

격자 탐색에서 반복적으로 쓰이는 '기계적인' 함수들을 모았습니다.
(노트북에서는 Node 클래스와 탐색 루프 등 '핵심 개념'에 집중할 수 있게)

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import math


def CalcGridIndex(position, min_pos, resolution):
    """실제 좌표 [m] -> 격자 인덱스 (반올림)"""
    return round((position - min_pos) / resolution)


def CalcGridPosition(index, min_pos, resolution):
    """격자 인덱스 -> 실제 좌표 [m]"""
    return index * resolution + min_pos


def NodeKey(node, x_width, min_x, min_y):
    """(ix, iy) -> 1D 정수 키. open_set / closed_set 의 dict key 로 사용."""
    return (node.y - min_y) * x_width + (node.x - min_x)


def GetMotionModel():
    """8방향 이동 모델: [dx, dy, cost]. 직선 1.0, 대각선 sqrt(2)."""
    return [
        [1, 0, 1.0], [-1, 0, 1.0], [0, 1, 1.0], [0, -1, 1.0],
        [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
        [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)],
    ]


def IsValid(node, obstacle_map, min_x, min_y, max_x, max_y, resolution):
    """node 가 격자 범위 안에 있고 장애물 셀이 아니면 True."""
    px = CalcGridPosition(node.x, min_x, resolution)
    py = CalcGridPosition(node.y, min_y, resolution)
    if px < min_x or py < min_y or px >= max_x or py >= max_y:
        return False
    return not obstacle_map[node.x][node.y]


def ReconstructPath(goal, closed_set, min_x, min_y, resolution):
    """goal 의 parent_index 체인을 따라 목표 -> 출발로 역추적 후 뒤집어 반환."""
    path_x = [CalcGridPosition(goal.x, min_x, resolution)]
    path_y = [CalcGridPosition(goal.y, min_y, resolution)]
    parent = goal.parent_index
    while parent != -1:
        node = closed_set[parent]
        path_x.append(CalcGridPosition(node.x, min_x, resolution))
        path_y.append(CalcGridPosition(node.y, min_y, resolution))
        parent = node.parent_index
    path_x.reverse()
    path_y.reverse()
    return path_x, path_y


def DijkstraSearch(sx, sy, gx, gy, ox, oy, resolution, robot_radius, Node):
    """
    비교용 Dijkstra 탐색 (Step 01에서 이미 배운 알고리즘).
    Node 클래스를 인자로 받아 노트북과 동일한 노드 타입을 사용한다.
    반환: path_x, path_y, closed_x, closed_y, explore_order
    """
    import heapq
    from grid_map import BuildObstacleMap
    obstacle_map, min_x, min_y, max_x, max_y, x_width, y_width = \
        BuildObstacleMap(ox, oy, resolution, robot_radius)

    start = Node(CalcGridIndex(sx, min_x, resolution),
                 CalcGridIndex(sy, min_y, resolution), 0.0, -1)
    goal = Node(CalcGridIndex(gx, min_x, resolution),
                CalcGridIndex(gy, min_y, resolution), 0.0, -1)

    open_set = {NodeKey(start, x_width, min_x, min_y): start}
    closed_set = {}
    pq = [(0.0, start)]
    explore_order = []

    while pq:
        _, current = heapq.heappop(pq)
        cid = NodeKey(current, x_width, min_x, min_y)
        if cid not in open_set:
            continue
        open_set.pop(cid); closed_set[cid] = current
        explore_order.append((CalcGridPosition(current.x, min_x, resolution),
                              CalcGridPosition(current.y, min_y, resolution), current.cost))
        if current.x == goal.x and current.y == goal.y:
            goal.parent_index = current.parent_index; goal.cost = current.cost
            break
        for dx, dy, cost in GetMotionModel():
            nb = Node(current.x + dx, current.y + dy, current.cost + cost, cid)
            nk = NodeKey(nb, x_width, min_x, min_y)
            if not IsValid(nb, obstacle_map, min_x, min_y, max_x, max_y, resolution):
                continue
            if nk in closed_set:
                continue
            if nk not in open_set or open_set[nk].cost > nb.cost:
                open_set[nk] = nb; heapq.heappush(pq, (nb.cost, nb))

    path_x, path_y = ReconstructPath(goal, closed_set, min_x, min_y, resolution)
    closed_x = [CalcGridPosition(n.x, min_x, resolution) for n in closed_set.values()]
    closed_y = [CalcGridPosition(n.y, min_y, resolution) for n in closed_set.values()]
    return path_x, path_y, closed_x, closed_y, explore_order
