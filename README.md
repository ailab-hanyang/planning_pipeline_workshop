# 자율주행 Planning Pipeline 실습 교육

## 교육 목표

Global Path Planning부터 CARLA 시뮬레이션까지, 자율주행 Planning Pipeline 전체 흐름을 실습한다.

---

## 커리큘럼

| Day | 폴더 | 내용 | 시간 |
|-----|------|------|------|
| Day 1 | `01_global_planning/` | Dijkstra, A*, Hybrid A* | 3h |
| Day 2 | `02_trajectory_planning/` | Cubic Spline, Longitudinal Motion Planning, Predefined Sample-based Planner, Optimization Planning | 4h |
| Day 3 | `03_carla_simulation/` | CARLA 구조·세팅, 기본 실습, Traffic Manager(+차선 변경), Planning(경로→궤적→주행 · TTC 제동 · 끼어들기 시나리오 테스트), 센서 실습(선택), AI Planning 개요 | - |

---

## 환경 설치

### Day 1-2
```bash
# 저장소 폴더(planning_pipeline_workshop)에서 실행
sudo apt update && sudo apt install -y python3.10-venv   # venv 도구 (관리자 비밀번호 필요, 이미 있으면 그대로 넘어감)
python3.10 -m venv .venv                                  # 실습 전용 파이썬 환경(.venv) 만들기 (처음 한 번)
source .venv/bin/activate                                 # 켜기: 새 터미널마다 다시 실행 (앞에 (.venv) 표시되면 켜진 것)
pip install -r requirements/requirements.txt
```

노트북 열기: VS Code 에서 `01_global_planning/step01_dijkstra.ipynb` 열기
→ 오른쪽 위 `Select Kernel` → `Python Environments` → `.venv` 선택

### Day 3 (CARLA 추가)
```bash
# Day 1-2 의 .venv 를 켠 상태에서 설치
# CARLA 클라이언트 설치 (경로는 설치 환경에 따라 다름)
pip install /opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl

pip install -r requirements/requirements_carla.txt
pip install -r /opt/carla-simulator/PythonAPI/examples/requirements.txt   # CARLA 공식 예제용 (numpy<2 로 맞춰짐, 실습 코드 동작 확인됨)
```

실습 PC 환경 점검 (수업 시작 시, 위 설치 후):
`bash env_check/check_env.sh --python .venv/bin/python`
→ 마지막 요약이 `FAIL 0` 이면 준비 완료 / FAIL 항목은 화면의 조치 안내대로 해결 후 다시 실행

> conda를 선호하는 경우 `requirements/environment.yml` 참고

CARLA 서버 실행 (Day 3 스크립트 실행 전): CARLA 설치 폴더에서 `./CarlaUE4.sh`  (서버 주소 `localhost:2000`)
- 주의: `-quality-level=Low` 옵션 사용 금지 → 맵을 바꿀 때 서버가 종료됨

Day 3 스크립트 실행 (서버를 켠 터미널과 다른 새 터미널에서):
```bash
cd 03_carla_simulation/1_CARLA_Basic        # 실습 폴더로 이동
source ../../.venv/bin/activate             # .venv 켜기 (앞에 (.venv) 표시 확인)
python3 P_Basic_1_connectserver.py          # 실행, 종료는 Ctrl+C (pygame 창이 뜨는 실습은 ESC)
```
- `P_Planning_3_cutin_test.py` 는 끼어드는 거리(m)를 인자로 받음: `python3 P_Planning_3_cutin_test.py 30` (생략 시 50)
- `4_CARLA_SensorPractice/` 는 선택 과제
Traffic Manager 실습(`2_CARLA_TrafficManager/P_TrafficManager_2~4`)은 spawn point 번호가 `Town10HD_Opt` 기준이라 실행 시 해당 맵을 자동으로 로드합니다.

---

## 폴더 구조

```
planning_pipeline_workshop/
├── 01_global_planning/       ← Day 1 실습 노트북
├── 02_trajectory_planning/   ← Day 2 실습 노트북
├── 03_carla_simulation/      ← Day 3 실습 스크립트 (1_Basic / 2_TrafficManager / 3_Planning / 4_SensorPractice)
├── utils/                    ← Day 1-2 공통 모듈 (맵·시각화·solver 실행부)
├── LICENSE                   ← MIT License
├── NOTICE.md                 ← 외부 코드(PythonRobotics) 라이선스 고지
├── 프로젝트_평가문제.docx    ← 평가 문제 제출 양식 (실습 화면 캡처 + 답변 작성)
└── requirements/             ← 환경 설치 파일
```

CARLA 공식 예제·유틸(`manual_control.py`, `generate_traffic.py` 등)은 CARLA 설치 폴더의 `PythonAPI/examples`, `PythonAPI/util` 에 있습니다.
