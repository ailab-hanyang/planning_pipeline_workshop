# 자율주행 Planning Pipeline 실습 교육

## 교육 목표

Global Path Planning부터 CARLA 시뮬레이션까지, 자율주행 Planning Pipeline 전체 흐름을 실습한다.

---

## 커리큘럼

| Day | 폴더 | 내용 | 시간 |
|-----|------|------|------|
| Day 1 | `01_global_planning/` | Dijkstra, A*, Hybrid A* | 3h |
| Day 2 | `02_trajectory_planning/` | Spline, Frenet, Velocity, Candidate, Cost, RRT | 4h |
| Day 3 | `03_carla_simulation/` | CARLA 기본, Traffic Manager, 판단 시나리오 | - |

---

## 환경 설치

### Day 1-2
```bash
python3.10 -m venv .venv
source .venv/bin/activate
pip install -r requirements/requirements.txt
```

### Day 3 (CARLA 추가)
```bash
# CARLA 클라이언트 설치 (경로는 설치 환경에 따라 다름)
pip install /opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-linux_x86_64.whl

pip install -r requirements/requirements_carla.txt
```

> conda를 선호하는 경우 `requirements/environment.yml` 참고

---

## 폴더 구조

```
planning_pipeline_workshop/
├── 01_global_planning/       ← Day 1 실습 노트북
├── 02_trajectory_planning/   ← Day 2 실습 노트북
├── 03_carla_simulation/      ← Day 3 실습 노트북 + scripts/
├── utils/                    ← 공통 모듈
├── scenarios/                ← 공통 맵/시나리오
├── references/               ← PythonRobotics 원본 (비교용)
├── solutions/                ← 강사용 완성본
└── requirements/             ← 환경 설치 파일
```
