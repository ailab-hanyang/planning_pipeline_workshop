# -*- coding: utf-8 -*-
"""
Constants, scenarios and visualization helpers for
step04_optimization_trajectory_planning notebook.

이 실습은 '최적화 기반 궤적 계획(optimization-based trajectory planning)' 을 다룬다.
  - 결정변수 z(조향 변화율 시퀀스) -> bicycle model -> 미래 trajectory
  - cost(soft) 와 constraint(hard) 로 "좋은 trajectory" 를 정의
  - SLSQP solver 가 합격(제약 만족) 계획 중 cost 최저인 계획을 탐색

개념 학습에 필요한 코드(bicycle model / cost / constraint / solver)는 노트북에 남기고,
그 외의 코드(공통 상수, 시나리오 생성, 도로·차량 그리기, 채점판, 각종 시각화)는
전부 이 모듈에 정의한다. 노트북에서는 import 해서 사용하기만 하면 된다.

Education : Hanyang University, Automotive Intelligence Laboratory (AILAB)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import matplotlib.font_manager as fm

from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
from scipy.optimize import minimize
from IPython.display import HTML


# ==================== 한글 폰트 ====================
def SetKoreanFont():
    """그림·표(스코어카드) 안의 한글이 □□ 로 깨지지 않도록 시스템의 한글 폰트를 찾아 등록한다."""
    available_fonts = {font.name for font in fm.fontManager.ttflist}

    # 1순위: OS 별 대표 한글 폰트 (폰트 이름이 정확히 일치하는 경우)
    font_candidates = ['Malgun Gothic', 'AppleGothic', 'NanumGothic', 'NanumBarunGothic',
                       'Noto Sans CJK KR', 'Noto Sans KR']
    chosen = next((name for name in font_candidates if name in available_fonts), None)

    # 2순위: 이름이 정확히 일치하지 않아도 CJK/Nanum 계열이면 한글 글리프를 포함한다.
    #        (예: 리눅스에서 'Noto Sans CJK KR' 이 'Noto Sans CJK JP' 라는 이름으로만 잡히는 경우)
    if chosen is None:
        cjk_fonts = sorted(name for name in available_fonts if 'CJK' in name or 'Nanum' in name)
        chosen = cjk_fonts[0] if cjk_fonts else None

    if chosen is not None:
        # 리스트로 지정 → 한글 폰트에 없는 기호(✓, ✗ 등)는 DejaVu Sans 로 자동 대체(fallback)
        plt.rcParams['font.family'] = [chosen, 'DejaVu Sans']
        print(f'한글 폰트 설정 완료: {chosen}')
    else:
        print('경고: 한글 폰트를 찾지 못했습니다. 그림 안의 한글이 □ 로 깨질 수 있습니다.')
        print('  - Ubuntu: sudo apt install fonts-nanum  설치 후, 커널을 재시작하고 이 셀을 다시 실행하세요.')
        print('  - Windows(Malgun Gothic) / Mac(AppleGothic) 은 기본 폰트가 자동 인식됩니다.')
    plt.rcParams['axes.unicode_minus'] = False   # 음수 부호(−) 깨짐 방지


# ==================== 공통 상수 (Theory 0) ====================
LANE_W = 3.5             # 한 차선의 폭 (lane width) [m]
NUM_LANES = 5            # 차선 개수
ROAD_LEN = 200.0         # 전체 도로 길이 [m]

ROAD_Y0 = 0.0            # 도로 아래쪽 경계의 y 좌표 [m]
ROAD_Y1 = NUM_LANES * LANE_W   # 도로 위쪽 경계의 y 좌표 [m]
LANE = [LANE_W * (i + 0.5) for i in range(NUM_LANES)]   # 각 차선 중심의 y 좌표 목록 [m]

dt = 0.2                 # 시뮬레이션 한 스텝의 시간 간격 Δt [s]
N = 50                   # 예측 구간의 스텝 수 (planning horizon length)
L_WB = 2.7               # 휠베이스(wheel base, 앞뒤 바퀴축 사이 거리) [m]
V_REF = 12.0             # 주행 속도 [m/s] (고정) — 이번 실습은 '횡방향(조향)'만 최적화

R_EGO = 1.3              # ego 차량을 덮는 근사원의 반지름 [m]
R_OBJ = 1.2              # 장애물을 덮는 근사원의 반지름 [m]
D_SAFE = 0.6             # 원 사이에 두는 추가 안전 여유거리 (safety distance) [m]
ROAD_MARGIN = 1.0        # 도로 경계 안쪽으로 남겨두는 여유 [m]
EGO_OFFSETS = [0.6, 3.4] # ego를 덮는 원들의 기준점 대비 전방 오프셋 [m]

# 횡방향 LMPC(Linear MPC, 선형 모델예측제어) 관련 상수
MAX_STEER  = 0.40     # 조향각 δ 한계 [rad]      (state 제약)
MAX_DSTEER = 4.00     # 조향 변화율 δ̇ 한계 [rad/s] (control input bound)
STEER_WEIGHT = 2.0    # 조향각 크기 δ² 의 (고정) weight
COMFORT_DSTEER = 1.20 # 채점 기준: 조향 변화율 |δ̇| 이 이보다 작아야 'Comfort' 통과


# ==================== 시나리오 (Theory 1) ====================
def LaneChangeReference(y_start, y_end, center_x, width):
    def Yref(x):
        transition = 0.5 * (1.0 + np.tanh((x - center_x) / width))
        return y_start + (y_end - y_start) * transition

    def Dyref(x):
        tanh_value = np.tanh((x - center_x) / width)
        transition_slope = 0.5 / width * (1.0 - tanh_value**2)
        return (y_end - y_start) * transition_slope

    return Yref, Dyref


def DoubleLaneChangeReference(y_start, y_mid, x1, x2, width):
    # y_start → y_mid → y_start 로 돌아오는 S자 이중 차선변경 (x1 에서 올라가고 x2 에서 내려옴)
    def Yref(x):
        up = 0.5 * (1.0 + np.tanh((x - x1) / width))
        down = 0.5 * (1.0 + np.tanh((x - x2) / width))
        return y_start + (y_mid - y_start) * (up - down)

    def Dyref(x):
        d_up = 0.5 / width * (1.0 - np.tanh((x - x1) / width) ** 2)
        d_down = 0.5 / width * (1.0 - np.tanh((x - x2) / width) ** 2)
        return (y_mid - y_start) * (d_up - d_down)

    return Yref, Dyref


def StraightReference(lane_id):
    lane_center = LANE[lane_id]

    def Yref(x):
        return lane_center * np.ones_like(x)

    def Dyref(x):
        return np.zeros_like(x)

    return Yref, Dyref


def BadReference():
    # x=35 부터 도로 밖으로 올라가다가 도로 위쪽 경계 3m 바깥에서 평평해지는 reference.
    # (끝없이 올라가면 '덜 전진할수록 오차가 작아지는' 지그재그 해가 최적이 되어 시연이 망가짐)
    top = ROAD_Y1 + 3.0

    def Yref(x):
        return np.minimum(LANE[1] + 0.28 * np.clip(x - 35.0, 0.0, None), top)

    def Dyref(x):
        return np.where((x > 35.0) & (LANE[1] + 0.28 * (x - 35.0) < top), 0.28, 0.0)

    return Yref, Dyref


# 시나리오별 한 줄 설명 (그림 제목 등에 사용)
SCENARIO_DESC = {
    'curve':   'L1→L3 차선변경 추종 (장애물 없음)',
    'avoid':   'L1 직진 + 전방 정지차 회피',
    'badref':  '도로 밖으로 향하는 나쁜 reference',
    'mission': '차선변경 + 회피 종합 미션',
    'scurve':  '(도전) S자 이중 차선변경 추종',
    'slalom':  '(도전) 정지차 2대 지그재그 회피',
    'wall':    '(도전) 두 차선 동시 봉쇄 → 크게 우회',
}

EGO_START = (12.0, LANE[1])   # 모든 시나리오에서 ego 는 L1, x=12 m 에서 출발


def MakeScenario(name):
    """시나리오 이름 -> (Yref, Dyref, obstacles, x_view). 기본 4가지 + 도전 3가지."""
    # ---------- 기본 시나리오 ----------
    # 1) curve  : 장애물 없이 L1 -> L3 로 부드럽게 차선변경 (목적: reference 추종)
    if name == 'curve':
        Yref, Dyref = LaneChangeReference(y_start=LANE[1], y_end=LANE[3], center_x=80.0, width=12.0)
        return Yref, Dyref, [], 145.0

    # 2) avoid  : L1 직진 reference + 같은 차선 앞 정지차 (목적: 직진 중 장애물 회피)
    if name == 'avoid':
        Yref, Dyref = StraightReference(lane_id=1)
        return Yref, Dyref, [(70.0, LANE[1])], 120.0

    # 3) badref : reference 자체가 도로 밖을 향함 (목적: hard constraint 보호 확인)
    if name == 'badref':
        Yref, Dyref = BadReference()
        return Yref, Dyref, [], 120.0

    # 4) mission: L1 -> L3 차선변경 + L3 앞 정지차 (목적: 추종 + 회피 + 안전 종합)
    if name == 'mission':
        Yref, Dyref = LaneChangeReference(y_start=LANE[1], y_end=LANE[3], center_x=75.0, width=11.0)
        return Yref, Dyref, [(118.0, LANE[3])], 150.0

    # ---------- 도전 시나리오 (추가 학습) ----------
    # 5) scurve : L1 -> L3 -> L1 S자 이중 차선변경 (목적: track ↔ comfort 트레이드오프 체감)
    if name == 'scurve':
        Yref, Dyref = DoubleLaneChangeReference(y_start=LANE[1], y_mid=LANE[3], x1=50.0, x2=95.0, width=9.0)
        return Yref, Dyref, [], 145.0

    # 6) slalom : L1 직진 + L1, L2 에 엇갈려 서 있는 정지차 2대 (목적: 연속 회피)
    if name == 'slalom':
        Yref, Dyref = StraightReference(lane_id=1)
        return Yref, Dyref, [(60.0, LANE[1]), (95.0, LANE[2])], 135.0

    # 7) wall   : L1 직진 + L1·L2 를 나란히 막은 정지차 2대 (목적: reference 잠시 포기, 크게 우회)
    if name == 'wall':
        Yref, Dyref = StraightReference(lane_id=1)
        return Yref, Dyref, [(95.0, LANE[1]), (95.0, LANE[2])], 135.0

    raise ValueError(f'unknown scenario: {name}')


def Thetaref(Dyref, x):
    """reference path 의 기울기 -> 목표 heading 각도."""
    return np.arctan(Dyref(x))


# ==================== 기하 보조 (충돌 제약용 원 중심) ====================
def ObstacleCircleCenters(obstacles):
    """장애물 차량 하나를 앞/중앙/뒤 3개의 원으로 근사한 중심 목록."""
    centers = []
    for obstacle_x, obstacle_y in obstacles:
        for offset_x in [-1.5, 0.0, 1.5]:                # 앞 / 중앙 / 뒤
            centers.append((obstacle_x + offset_x, obstacle_y))
    return centers


def EgoCircleCenters(states):
    """ego 차량을 덮는 원들의 중심 궤적 (heading 방향으로 EGO_OFFSETS 만큼 전방)."""
    centers = []
    for offset in EGO_OFFSETS:
        x_center = states[:, 0] + offset * np.cos(states[:, 2])
        y_center = states[:, 1] + offset * np.sin(states[:, 2])
        centers.append(np.stack([x_center, y_center], axis=1))
    return centers


# ==================== engine 연결 + solver ====================
# 노트북에서 완성하는 4개 함수(KinematicBicycleStep / Simulate / CostFn / ConsFn)를
# RegisterEngine 으로 연결하면, 이 모듈의 Optimize / Run / ShowWeightSweep 이 동작한다.
_ENGINE = {}


def RegisterEngine(step_fn, simulate_fn, cost_fn, cons_fn):
    """노트북에서 완성한 model · cost · constraint 구현을 solver 와 시각화 도구에 연결한다."""
    _ENGINE.update(step=step_fn, simulate=simulate_fn, cost=cost_fn, cons=cons_fn)
    print('engine 연결 완료 — Optimize() / Run() / ShowWeightSweep() 사용 가능')


def _Engine(key):
    if key not in _ENGINE:
        raise RuntimeError("아직 engine 이 연결되지 않았습니다. "
                           "Theory 5 셀의 RegisterEngine(...) 을 먼저 실행하세요.")
    return _ENGINE[key]


def NeutralInit(initial_state):
    """가장 단순한 초기 추정: δ̇ = 0 (그냥 직진)."""
    return np.zeros(N)


def WarmStart(initial_state, Yref, Dyref):
    """reference 를 대충 따라가는 비례 제어기로 초기 δ̇ 시퀀스를 만든다 (solver warm start)."""
    step = _Engine('step')
    state = np.array(initial_state, dtype=float)
    controls = np.zeros(N)
    for k in range(N):
        x, y, theta, delta = state
        theta_ref = np.arctan(Dyref(x))
        lateral_error = float(Yref(x)) - y
        delta_desired = (theta_ref - theta) + np.arctan2(0.5 * lateral_error, V_REF)
        delta_desired = np.clip(delta_desired, -MAX_STEER, MAX_STEER)
        dsteer = np.clip((delta_desired - delta) / dt, -MAX_DSTEER, MAX_DSTEER)
        controls[k] = dsteer
        state = step(state, dsteer)
    return controls


def SolveTrajectoryOptimization(weights, scenario, use_collision=True, use_road=True,
                                init='warm', record_history=False):
    """SciPy SLSQP 로 '제약을 만족하면서 cost 최소'인 조향 계획을 찾는다."""
    cost_fn = _Engine('cost')
    cons_fn = _Engine('cons')
    Yref, Dyref, obstacles, x_view = MakeScenario(scenario)
    initial_state = np.array([12.0, LANE[1], 0.0, 0.0])   # x, y, θ, δ(=0, 직진)

    if init == 'warm':
        initial_guess = WarmStart(initial_state, Yref, Dyref)   # reference 추종 초기 추정
    else:
        initial_guess = NeutralInit(initial_state)              # 직진(δ̇=0) 초기 추정

    # 반복 과정 기록: iteration 0 = 초기 추정, 이후 SLSQP 가 한 번 반복할 때마다 저장
    history = [np.array(initial_guess, dtype=float)]
    callback = (lambda z: history.append(np.array(z, dtype=float))) if record_history else None

    bounds = [(-MAX_DSTEER, MAX_DSTEER)] * N              # control input(δ̇) 한계
    constraints = [{
        'type': 'ineq',
        'fun': lambda z: cons_fn(z, obstacles, initial_state, use_collision, use_road),
    }]

    # cost 를 초기 추정값 기준으로 정규화 (값 크기가 제약(m)과 비슷해야 SLSQP 가 안정적으로 수렴).
    # 상수로 나누는 것이라 최적해는 같다.
    cost_scale = max(1.0, cost_fn(initial_guess, weights, Yref, Dyref, initial_state))
    result = minimize(
        fun=lambda z: cost_fn(z, weights, Yref, Dyref, initial_state) / cost_scale,
        x0=initial_guess,
        method='SLSQP',
        bounds=bounds,
        constraints=constraints,
        options={'maxiter': 400, 'ftol': 1e-7},
        callback=callback,
    )

    context = {'yref': Yref, 'dyref': Dyref, 'obs': obstacles, 'xview': x_view,
               'x0': initial_state, 'use_collision': use_collision,
               'use_road': use_road, 'W': weights,
               'history': history if record_history else []}
    return result, context


def Optimize(weights, scenario, use_collision=True, use_road=True,
             init='warm', record_history=False):
    """solver 실행 -> 결과를 (trajectory, control, 채점용 정보) 딕셔너리로 정리해 반환."""
    simulate = _Engine('simulate')
    cost_fn = _Engine('cost')
    result, context = SolveTrajectoryOptimization(
        weights, scenario, use_collision, use_road, init, record_history)

    states = simulate(result.x, context['x0'])
    controls = np.asarray(result.x).reshape(-1)
    # 제약 최대 위반량 [m] (0 이면 모든 제약 만족). 채점 허용치(0.05 m)를 넘으면 Run() 이 경고한다.
    max_violation = max(0.0, -float(np.min(_Engine('cons')(
        result.x, context['obs'], context['x0'], use_collision, use_road))))

    history_states = [simulate(z, context['x0']) for z in context['history']]
    history_costs = [cost_fn(z, weights, context['yref'], context['dyref'], context['x0'])
                     for z in context['history']]

    return {'s': states, 'u': controls, 'yref': context['yref'], 'dyref': context['dyref'],
            'obs': context['obs'], 'xview': context['xview'], 'scenario': scenario,
            'W': weights,
            'objective': cost_fn(result.x, weights, context['yref'], context['dyref'], context['x0']),
            'solver_success': result.success, 'solver_message': result.message,
            'max_violation': max_violation,
            'use_collision': use_collision, 'use_road': use_road,
            'history_states': history_states, 'history_costs': history_costs}


# ==================== 도로 · 차량 그리기 ====================
class Car:
    def __init__(
        self,
        x,
        y,
        theta=0.0,
        color='#8a8f96',
        length=4.6,
        width=2.0,
        name='car',
    ):
        self.x = x
        self.y = y
        self.theta = theta
        self.color = color
        self.length = length
        self.width = width
        self.name = name

    def circles(self, n=3):
        offsets = np.linspace(0.6, self.length - 0.6, n)
        x_centers = self.x + offsets * np.cos(self.theta)
        y_centers = self.y + offsets * np.sin(self.theta)

        return np.stack([x_centers, y_centers], axis=1)

    @property
    def radius(self):
        return self.width / 2.0 + 0.1


def DrawRoad(ax, x_view=(0.0, ROAD_LEN), ids=True):
    x_min = x_view[0]
    x_max = x_view[1]

    ax.set_facecolor('#2f343a')
    ax.add_patch(
        Rectangle(
            (x_min, ROAD_Y0),
            x_max - x_min,
            ROAD_Y1,
            facecolor='#454b52',
            zorder=0,
        )
    )
    ax.plot([x_min, x_max], [ROAD_Y0, ROAD_Y0], color='#e8c100', lw=3, zorder=1)
    ax.plot([x_min, x_max], [ROAD_Y1, ROAD_Y1], color='white', lw=3, zorder=1)

    for lane_id in range(1, NUM_LANES):
        lane_y = lane_id * LANE_W
        ax.plot(
            [x_min, x_max],
            [lane_y, lane_y],
            color='white',
            lw=1.5,
            ls=(0, (14, 12)),
            alpha=0.9,
            zorder=1,
        )

    if ids:
        for lane_id, lane_center in enumerate(LANE):
            ax.text(
                x_min + 2.0,
                lane_center,
                f'L{lane_id}',
                color='white',
                fontsize=8,
                va='center',
                alpha=0.5,
            )
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(ROAD_Y0 - 1.5, ROAD_Y1 + 1.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')


def DrawCar(ax, car, steer=0.0, alpha=0.95, circles=False):
    transform = mtransforms.Affine2D().rotate_around(
        car.x,
        car.y,
        car.theta,
    ) + ax.transData

    body = FancyBboxPatch(
        (car.x, car.y - car.width / 2.0),
        car.length,
        car.width,
        boxstyle='round,pad=0.02,rounding_size=0.5',
        lw=1.1,
        edgecolor='black',
        facecolor=car.color,
        alpha=alpha,
        transform=transform,
        zorder=5,
    )
    ax.add_patch(body)

    cabin = Rectangle(
        (car.x + car.length * 0.58, car.y - car.width * 0.32),
        car.length * 0.2,
        car.width * 0.64,
        facecolor='#bfe3ff',
        edgecolor='none',
        alpha=0.85,
        transform=transform,
        zorder=6,
    )
    ax.add_patch(cabin)

    wheel_specs = [
        (car.x, car.y - car.width / 2.0, False),
        (car.x, car.y + car.width / 2.0, False),
        (car.x + L_WB, car.y - car.width / 2.0, True),
        (car.x + L_WB, car.y + car.width / 2.0, True),
    ]

    for wheel_x, wheel_y, is_front_wheel in wheel_specs:
        wheel_transform = transform

        if is_front_wheel and abs(steer) > 1e-6:
            wheel_transform = (
                mtransforms.Affine2D().rotate_around(wheel_x, wheel_y, steer)
                .rotate_around(car.x, car.y, car.theta)
                + ax.transData
            )

        wheel = Rectangle(
            (wheel_x - 0.37, wheel_y - 0.14),
            0.75,
            0.28,
            facecolor='#111111',
            transform=wheel_transform,
            zorder=7,
        )
        ax.add_patch(wheel)

    if car.name == 'ego':
        ax.add_patch(Circle((car.x, car.y), 0.18, color='red', zorder=8))

    if circles:
        for center_x, center_y in car.circles():
            ax.add_patch(
                Circle(
                    (center_x, center_y),
                    car.radius,
                    fill=False,
                    edgecolor='red',
                    ls=':',
                    lw=1,
                    zorder=8,
                )
            )


# ==================== 시나리오 미리 보기 ====================
def PlotReferencePathPreview(scenario_names=None):
    if scenario_names is None:
        scenario_names = ['curve', 'avoid', 'badref', 'mission']

    fig, axes = plt.subplots(len(scenario_names), 1,
                             figsize=(12, 2.1 * len(scenario_names)), constrained_layout=True)

    if len(scenario_names) == 1:
        axes = [axes]

    for ax, scenario_name in zip(axes, scenario_names):
        Yref, Dyref, obstacles, x_view = MakeScenario(scenario_name)
        x_values = np.linspace(0.0, x_view, 500)
        y_values = Yref(x_values)

        ax.axhspan(ROAD_Y0, ROAD_Y1, facecolor='#454b52', alpha=0.35)
        ax.plot([0.0, x_view], [ROAD_Y0, ROAD_Y0], color='#e8c100', lw=2)
        ax.plot([0.0, x_view], [ROAD_Y1, ROAD_Y1], color='black', lw=1.5)

        for lane_index in range(1, NUM_LANES):
            lane_y = lane_index * LANE_W
            ax.plot([0.0, x_view], [lane_y, lane_y], color='gray', lw=1.0, ls='--', alpha=0.7)

        for lane_index, lane_center in enumerate(LANE):
            ax.text(2.0, lane_center, f'L{lane_index}', va='center', fontsize=8, color='dimgray')

        ax.plot(x_values, y_values, '--', color='limegreen', lw=2.5, label='reference path (목표 경로)')

        ax.scatter(*EGO_START, s=140, marker='>', color='#1f77b4',
                   edgecolor='black', zorder=6, label='ego 시작 위치')

        for obstacle_x, obstacle_y in obstacles:
            ax.scatter(obstacle_x, obstacle_y, s=120, marker='s', color='#d9534f',
                       edgecolor='black', zorder=5, label='장애물')

        ax.set_title(f'scenario = {scenario_name} — {SCENARIO_DESC[scenario_name]}')
        ax.set_xlim(0.0, x_view)
        ax.set_ylim(ROAD_Y0 - 1.0, ROAD_Y1 + 1.0)
        ax.set_ylabel('y [m]')
        ax.grid(True, alpha=0.2)

        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        ax.legend(unique.values(), unique.keys(), loc='upper right', fontsize=8)

    axes[-1].set_xlabel('x [m]')
    plt.show()


# ==================== 간단 trajectory 플롯 ====================
def PlotPlans(plans, scenario, title=None):
    """간단한 도로 배경 위에 여러 계획(trajectory)을 겹쳐 그린다.

    인자:
      plans    : (states, label, color) 튜플의 리스트 — states 는 Simulate() 결과
      scenario : 배경으로 쓸 시나리오 이름 (reference path 와 장애물 표시)
    """
    Yref, _, obstacles, x_view = MakeScenario(scenario)

    plt.figure(figsize=(11, 3.0))
    plt.axhspan(ROAD_Y0, ROAD_Y1, color='#eaeaea')
    for lane_id in range(1, NUM_LANES):
        plt.plot([0, x_view], [lane_id * LANE_W] * 2, color='gray', ls='--', lw=0.8, alpha=0.6)

    reference_x = np.linspace(0.0, x_view, 400)
    plt.plot(reference_x, Yref(reference_x), '--', color='limegreen', lw=2.2,
             label='reference path (목표 경로)')
    for obstacle_x, obstacle_y in obstacles:
        plt.scatter(obstacle_x, obstacle_y, s=150, marker='s', color='#d9534f',
                    edgecolor='black', zorder=5, label='정지차')

    for states, label, color in plans:
        plt.plot(states[:, 0], states[:, 1], color=color, lw=2.4, label=label)

    plt.xlim(0, x_view)
    plt.ylim(ROAD_Y0 - 1, ROAD_Y1 + 1)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    if title:
        plt.title(title)
    handles, labels = plt.gca().get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    plt.legend(unique.values(), unique.keys(), loc='upper left', fontsize=8)
    plt.tight_layout()
    plt.show()


# ==================== 채점(scorecard) + Run ====================
def Evaluate(res):
    states = res['s']
    controls = res['u']           # δ̇ 시퀀스 (control input)
    Yref = res['yref']
    obstacles = res['obs']

    tracking_mask = np.ones(len(states), dtype=bool)
    for obstacle_x, _ in obstacles:
        tracking_mask = tracking_mask & (np.abs(states[:, 0] - obstacle_x) > 12.0)
    tracking_error = states[tracking_mask, 1] - Yref(states[tracking_mask, 0])
    tracking_rmse = np.sqrt(np.mean(tracking_error**2))

    road_violation = max(
        0.0,
        (ROAD_Y0 + ROAD_MARGIN) - states[:, 1].min(),
        states[:, 1].max() - (ROAD_Y1 - ROAD_MARGIN),
    )

    if obstacles:
        clearances = []
        for ego_centers in EgoCircleCenters(states):
            for obstacle_x, obstacle_y in ObstacleCircleCenters(obstacles):
                dx = ego_centers[:, 0] - obstacle_x
                dy = ego_centers[:, 1] - obstacle_y
                distance = np.sqrt(dx**2 + dy**2)
                clearances.append(distance.min() - (R_EGO + R_OBJ))
        minimum_clearance = min(clearances)
    else:
        minimum_clearance = 99.0

    # comfort = 조향 변화율 |δ̇| (control input) 의 최댓값
    steering_rate = np.max(np.abs(controls)) if len(controls) else 0.0

    return {
        'Tracking': (
            tracking_rmse < 1.0,
            f'RMSE={tracking_rmse:.2f} m',
        ),
        'On-road': (
            road_violation < 0.05,
            f'out={road_violation:.2f} m',
        ),
        'No-collision': (
            (minimum_clearance > -0.05) if obstacles else True,
            f'clear={minimum_clearance:+.2f} m' if obstacles else 'no obstacle',
        ),
        'Comfort': (
            steering_rate < COMFORT_DSTEER,
            f'max|dsteer|={steering_rate:.2f} rad/s',
        ),
    }


def VisualizePlan(res, title=None, snap=6):
    states = res['s']
    Yref = res['yref']
    obstacles = res['obs']
    x_view = res['xview']

    checks = Evaluate(res)
    num_passed = sum(passed for passed, _ in checks.values())
    passed_all = num_passed == 4

    fig = plt.figure(figsize=(15, 5.4))
    grid = fig.add_gridspec(2, 1, height_ratios=[4, 1.0], hspace=0.45)
    ax = fig.add_subplot(grid[0])
    score_ax = fig.add_subplot(grid[1])
    score_ax.axis('off')

    DrawRoad(ax, x_view=(0.0, x_view))

    reference_x = np.linspace(0.0, x_view, 400)
    ax.plot(reference_x, Yref(reference_x), '--', color='lime', lw=2.2,
            zorder=2, label='reference path (목표 경로)')

    for obstacle_x, obstacle_y in obstacles:
        obstacle_car = Car(x=obstacle_x - 2.3, y=obstacle_y, theta=0.0,
                           color='#d9534f', name='obs')
        DrawCar(ax, obstacle_car, circles=True)

    ax.plot(states[:, 0], states[:, 1], color='cyan', lw=2.4, zorder=3,
            label='planned trajectory (계획된 궤적)')

    for step in range(0, len(states), snap):
        ego_car = Car(x=states[step, 0], y=states[step, 1], theta=states[step, 2],
                      color='#1f77b4', name='ego')
        DrawCar(ax=ax, car=ego_car, steer=states[step, 3], alpha=0.9)   # δ = state[3]

    ax.legend(loc='upper left', fontsize=9)

    banner = 'MISSION COMPLETE' if passed_all else 'MISSION FAILED'
    banner = f'{banner}  ({num_passed}/4)'
    if title:
        banner = f'{title}   —   {banner}'
    ax.set_title(banner, color=('#1a9850' if passed_all else '#d62728'),
                 fontsize=15, fontweight='bold')

    check_labels_kr = {'Tracking': ('경로 추종', '경로 이탈'), 'On-road': ('도로 이탈 없음', '도로 이탈'),
                       'No-collision': ('충돌 없음', '충돌 발생'), 'Comfort': ('승차감 양호', '급조향')}
    for index, (name, (is_ok, text)) in enumerate(checks.items()):
        box_x = index / 4.0 + 0.01
        score_ax.add_patch(Rectangle((box_x, 0.05), 0.23, 0.9,
                           transform=score_ax.transAxes,
                           facecolor=('#1a9850' if is_ok else '#d62728'), alpha=0.85))
        score_ax.text(box_x + 0.115, 0.74, f"{'✓' if is_ok else '✗'} {name}",
                      transform=score_ax.transAxes, ha='center', va='center',
                      color='white', fontsize=11, fontweight='bold')
        score_ax.text(box_x + 0.115, 0.47, check_labels_kr.get(name, ('', ''))[0 if is_ok else 1],
                      transform=score_ax.transAxes, ha='center', va='center',
                      color='white', fontsize=8.5)
        score_ax.text(box_x + 0.115, 0.21, text, transform=score_ax.transAxes,
                      ha='center', va='center', color='white', fontsize=9)

    plt.show()
    return checks


def Run(W, scenario='mission', use_collision=True, use_road=True, title=None, snap=6):
    """Optimize 실행 -> 결과 그림 + 채점판. res 딕셔너리를 반환한다."""
    res = Optimize(weights=W, scenario=scenario,
                   use_collision=use_collision, use_road=use_road)
    if res['max_violation'] > 0.05:
        print(f"[solver 경고] 제약을 모두 만족하는 해를 찾지 못했습니다 (최대 위반 {res['max_violation']:.2f} m, "
              f"{res['solver_message']}). 아래 그림은 solver 의 마지막 반복값입니다.")
    VisualizePlan(res, title=title, snap=snap)
    return res


# ==================== 프로파일 · 애니메이션 ====================
def ShowProfiles(res):
    states = res['s']
    controls = res['u']                              # δ̇ (control input)
    control_time = np.arange(len(controls)) * dt
    state_time = np.arange(len(states)) * dt

    fig, axes = plt.subplots(1, 3, figsize=(14, 2.6))

    axes[0].plot(control_time, controls)                 # rad/s: 채점판(Comfort 기준 1.20 rad/s)과 같은 단위
    comfort_limit = COMFORT_DSTEER
    axes[0].axhline(comfort_limit, color='#d62728', ls='--', lw=1.0, label='Comfort 통과 기준')
    axes[0].axhline(-comfort_limit, color='#d62728', ls='--', lw=1.0)
    axes[0].set_title('control input: 조향 변화율 δ̇ [rad/s]')
    axes[0].legend(fontsize=7, loc='upper right')

    axes[1].plot(state_time, np.degrees(states[:, 3]))
    steer_limit = np.degrees(MAX_STEER)
    axes[1].axhline(steer_limit, color='#d62728', ls='--', lw=1.0, label='조향각 한계 (hard)')
    axes[1].axhline(-steer_limit, color='#d62728', ls='--', lw=1.0)
    axes[1].set_title('state: 조향각 δ [deg]')
    axes[1].legend(fontsize=7, loc='upper right')

    axes[2].plot(states[:, 0], states[:, 1], label='계획된 y')
    ref_x = states[:, 0]
    axes[2].plot(ref_x, res['yref'](ref_x), '--', color='gray', label='y_ref (목표)')
    axes[2].set_title('횡방향 위치 y(x)'); axes[2].legend(fontsize=8)
    axes[2].set_xlabel('x [m]')

    for ax in (axes[0], axes[1]):
        ax.set_xlabel('t [s]')
    for ax in axes:
        ax.grid(True)

    plt.tight_layout()
    plt.show()


def Animate(res, frame_step=2):
    states = res['s']
    obstacles = res['obs']
    Yref = res['yref']

    fig, ax = plt.subplots(figsize=(11, 3.3))

    def update(frame_index):
        ax.clear()
        center_x = states[frame_index, 0]
        DrawRoad(ax, x_view=(center_x - 28.0, center_x + 40.0), ids=False)

        reference_x = np.linspace(center_x - 28.0, center_x + 40.0, 200)
        ax.plot(reference_x, Yref(reference_x), '--', color='lime', lw=2)

        for obstacle_x, obstacle_y in obstacles:
            obstacle_car = Car(x=obstacle_x - 2.3, y=obstacle_y, theta=0.0,
                               color='#d9534f', name='obs')
            DrawCar(ax, obstacle_car)

        ego_car = Car(x=states[frame_index, 0], y=states[frame_index, 1],
                      theta=states[frame_index, 2], color='#1f77b4', name='ego')
        DrawCar(ax=ax, car=ego_car, steer=states[frame_index, 3])   # δ = state[3]

        ax.plot(states[:frame_index + 1, 0], states[:frame_index + 1, 1],
                color='cyan', lw=1.8)
        ax.set_title(f't={frame_index * dt:.1f}s   δ={np.degrees(states[frame_index, 3]):+.1f}°')

    animation = FuncAnimation(fig, update, frames=range(0, len(states), frame_step), interval=70)
    plt.close(fig)
    return HTML(animation.to_jshtml())


# ==================== 최적화 반복 과정 시각화 ====================
def ShowOptimizationProgress(res, max_lines=12):
    """Optimize(..., record_history=True) 결과에서 iteration 별 궤적 변화와 cost 감소를 그린다."""
    history_states = res['history_states']
    history_costs = res['history_costs']
    if len(history_states) < 2:
        print("기록된 반복 과정이 없습니다. Optimize(..., record_history=True) 로 실행하세요.")
        return

    num_records = len(history_states)          # 초기 추정 + 반복 횟수
    # 반복이 많으면 균등 간격으로 추려서 표시 (초기 추정과 최종 해는 항상 포함)
    if num_records > max_lines:
        shown = np.unique(np.linspace(0, num_records - 1, max_lines).astype(int))
    else:
        shown = np.arange(num_records)

    fig = plt.figure(figsize=(15, 4.0))
    grid = fig.add_gridspec(1, 2, width_ratios=[2.6, 1.0], wspace=0.25)
    ax = fig.add_subplot(grid[0])
    cost_ax = fig.add_subplot(grid[1])

    x_view = res['xview']
    ax.axhspan(ROAD_Y0, ROAD_Y1, color='#eaeaea')
    for lane_id in range(1, NUM_LANES):
        ax.plot([0, x_view], [lane_id * LANE_W] * 2, color='gray', ls='--', lw=0.8, alpha=0.6)

    reference_x = np.linspace(0.0, x_view, 400)
    ax.plot(reference_x, res['yref'](reference_x), '--', color='limegreen', lw=2.2,
            label='reference path (목표 경로)')
    for obstacle_x, obstacle_y in res['obs']:
        ax.scatter(obstacle_x, obstacle_y, s=150, marker='s', color='#d9534f',
                   edgecolor='black', zorder=6, label='장애물')

    colors = plt.cm.Blues(np.linspace(0.35, 1.0, len(shown)))
    alphas = np.linspace(0.5, 1.0, len(shown))
    for color, alpha, iteration in zip(colors, alphas, shown):
        states = history_states[iteration]
        is_final = (iteration == num_records - 1)
        if iteration == 0:
            label = 'iteration 0 (초기 추정)'
        elif is_final:
            label = f'iteration {iteration} (최종 해)'
        else:
            label = None
        ax.plot(states[:, 0], states[:, 1], color=color,
                lw=2.8 if is_final else 1.3, alpha=1.0 if is_final else alpha,
                zorder=5 if is_final else 3, label=label)

    ax.set_xlim(0, x_view)
    ax.set_ylim(ROAD_Y0 - 1, ROAD_Y1 + 1)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'반복마다 점진적으로 개선되는 trajectory (총 {num_records - 1}회 반복)')
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc='upper left', fontsize=8)

    cost_ax.plot(np.arange(num_records), history_costs, marker='o', ms=3.5, color='#1f77b4')
    cost_ax.set_yscale('log')
    cost_ax.set_xlabel('iteration')
    cost_ax.set_ylabel('cost J (log)')
    cost_ax.set_title('cost 감소 과정')
    cost_ax.grid(True, alpha=0.3)
    plt.show()


# ==================== weight sweep 시각화 ====================
def ShowWeightSweep(param, values, base_weights, scenario='scurve', title=None):
    """weight 하나만 조금씩 바꿔 가며 최적화를 반복 -> trajectory 의 점진적 변화 관찰."""
    Yref, Dyref, obstacles, x_view = MakeScenario(scenario)

    fig = plt.figure(figsize=(13, 6.0))
    grid = fig.add_gridspec(2, 2, height_ratios=[1.5, 1.0], hspace=0.45, wspace=0.22)
    road_ax = fig.add_subplot(grid[0, :])
    deviation_ax = fig.add_subplot(grid[1, 0])
    steer_ax = fig.add_subplot(grid[1, 1])

    DrawRoad(road_ax, x_view=(0.0, x_view))
    reference_x = np.linspace(0.0, x_view, 400)
    road_ax.plot(reference_x, Yref(reference_x), '--', color='lime', lw=2.0,
                 label='reference path (목표 경로)')
    for obstacle_x, obstacle_y in obstacles:
        DrawCar(road_ax, Car(x=obstacle_x - 2.3, y=obstacle_y, color='#d9534f', name='obs'))

    colors = plt.cm.plasma(np.linspace(0.15, 0.85, len(values)))
    for color, value in zip(colors, values):
        weights = dict(base_weights)
        weights[param] = value
        res = Optimize(weights=weights, scenario=scenario)
        states, controls = res['s'], res['u']

        road_ax.plot(states[:, 0], states[:, 1], color=color, lw=1.8,
                     label=f'{param} = {value:g}')
        deviation = states[:, 1] - Yref(states[:, 0])
        deviation_ax.plot(states[:, 0], deviation, color=color, lw=1.8)
        steer_ax.plot(np.arange(len(controls)) * dt, controls,
                      color=color, lw=1.8)

    road_ax.legend(loc='upper left', fontsize=8, ncol=2)
    road_ax.set_title(title or f"'{param}' weight 에 따른 trajectory 의 점진적 변화 ({scenario})")

    deviation_ax.axhline(0.0, color='limegreen', ls='--', lw=1.5)
    deviation_ax.set_title('reference 와의 횡방향 편차 y − y_ref [m]', fontsize=10)
    deviation_ax.set_xlabel('x [m]')
    deviation_ax.set_ylabel('편차 [m]')
    deviation_ax.grid(True, alpha=0.3)

    steer_ax.set_title('control input: 조향 변화율 δ̇ [rad/s]', fontsize=10)
    steer_ax.set_xlabel('t [s]')
    steer_ax.grid(True, alpha=0.3)

    plt.show()
