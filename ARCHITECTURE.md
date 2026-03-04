# 🏗️ Swarm Formation 아키텍처 (리팩토링 후)

## 📦 패키지 구조

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────────────┐      ┌──────────────────────┐    │
│  │ formation_manager    │      │ trajectory_planner   │    │
│  │ (대형 관리)           │─────▶│ (궤적 생성)           │    │
│  │                      │ msg  │                      │    │
│  │ - YAML 시나리오      │      │ - TrajectoryCommand  │    │
│  │ - SwarmGraph         │      │   수신               │    │
│  │ - Hungarian          │      │ - 궤적 계획          │    │
│  │ - 드론별 목표 계산   │      │ - 궤적 최적화        │    │
│  └──────────────────────┘      └──────────────────────┘    │
│           │                              │                  │
│           │ TrajectoryCommand            │ PolyTraj         │
│           ↓                              ↓                  │
└─────────────────────────────────────────────────────────────┘
                                           │
┌─────────────────────────────────────────┼───────────────────┐
│                  Library Layer           │                   │
├──────────────────────────────────────────┼───────────────────┤
│                                          │                   │
│  path_manager (라이브러리 + 메시지)      │                   │
│  ┌────────────────────────────────────┐ │                   │
│  │ - path_manager.cpp                 │ │                   │
│  │ - polynomial_traj.cpp              │◀┘                   │
│  │ - uniform_bspline.cpp              │                     │
│  │ - msg/PolyTraj.msg                 │                     │
│  └────────────────────────────────────┘                     │
│           │                                                  │
│           │ depends on                                       │
│           ↓                                                  │
│  ┌────────────────┬──────────────────┬────────────────┐    │
│  │ path_optimizer │  path_planner    │  swarm_graph   │    │
│  │ (LBFGS 최적화) │  (A* 경로 계획) │  (대형 유사도) │    │
│  └────────────────┴──────────────────┴────────────────┘    │
└──────────────────────────────────────────────────────────────┘
                           │
┌──────────────────────────┼───────────────────────────────────┐
│         Hardware Interface Layer                             │
├──────────────────────────┼───────────────────────────────────┤
│                          │                                   │
│  ┌─────────────────┐     │     ┌─────────────────────┐      │
│  │  traj_server    │◀────┘     │  rover_control      │      │
│  │  (샘플링 @10ms) │           │  (MAVLink 제어)     │      │
│  └─────────────────┘           └─────────────────────┘      │
└──────────────────────────────────────────────────────────────┘
                           │
┌──────────────────────────┼───────────────────────────────────┐
│              Physical / Simulation Layer                     │
│              (PX4 Autopilot / RViz)                          │
└──────────────────────────────────────────────────────────────┘
```

---

## 🔀 데이터 흐름

### 전체 시스템 흐름
```
1. formation_manager
   ├─ scenario_default.yaml 로드
   ├─ 현재 드론 위치 수신 (/V{N}/odom)
   ├─ 거리/유사도 조건 확인
   ├─ 대형 패턴 생성 (FormationUtils)
   ├─ 헝가리안 알고리즘으로 최적 할당
   └─ 각 드론에 개별 TrajectoryCommand 발행
       ↓ /V1/trajectory_command
       ↓ /V2/trajectory_command
       ↓ /V3/trajectory_command
       ↓ /V4/trajectory_command

2. trajectory_planner (드론별 독립)
   ├─ TrajectoryCommand 수신
   │   ├─ target_position (개인 목표)
   │   ├─ waypoints[] (경유점)
   │   └─ formation_offset (대형 오프셋)
   ├─ PathManager::planGlobalTraj()
   │   └─ A* 경로 계획
   ├─ PathManager::computeAndOptimizePath()
   │   └─ PolyTrajOptimizer (LBFGS)
   └─ PolyTraj 발행
       ↓ /V{N}/planning/trajectory

3. traj_server
   ├─ PolyTraj 수신
   ├─ 10ms 주기 샘플링
   └─ PositionCommand 발행
       ↓ /agent{N}/target_position

4. rover_control
   ├─ PositionCommand 수신
   └─ MAVLink 제어 신호 생성
```

---

## 📨 메시지 정의

### formation_msgs/TrajectoryCommand
```
드론별 개별 궤적 명령 (formation_manager → trajectory_planner)

std_msgs/Header header
int32 drone_id                         # 0, 1, 2, 3
int32 sequence                         # 미션 시퀀스
string mission_id                      # "mission_0", "mission_1"

geometry_msgs/Point target_position    # 개인 목표 (이미 할당됨)
geometry_msgs/Vector3 target_velocity
geometry_msgs/Point[] waypoints        # 경유점들

string formation_type                  # "square", "triangle"
float64 formation_scale                # 2.0, 3.0
geometry_msgs/Vector3 formation_offset # 대형 내 상대 위치
```

### path_manager/msg/PolyTraj
```
다항식 궤적 (trajectory_planner → traj_server)

int16 drone_id
builtin_interfaces/Time start_time
float64[] duration                     # 각 piece의 duration
float64[] coef_x                       # x 계수
float64[] coef_y                       # y 계수
float64[] coef_z                       # z 계수
```

### path_manager/msg/PositionCommand
```
위치 명령 (traj_server → rover_control)

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
```

---

## 🎯 각 패키지 역할

### formation_msgs (메시지)
- **역할**: 메시지 정의만
- **의존성**: std_msgs, geometry_msgs
- **노드**: 없음

### formation_manager (대형 관리)
- **역할**:
  - YAML 시나리오 로드
  - 대형 전환 시점 판단 (거리/유사도)
  - 대형 패턴 생성
  - 헝가리안 알고리즘
  - 드론별 개별 명령 발행
- **의존성**: swarm_graph, formation_msgs
- **발행**: /V{N}/trajectory_command
- **구독**: /V{N}/odom (위치 추적용)

### trajectory_planner (궤적 생성)
- **역할**:
  - 개인 목표 수신
  - A* 경로 계획
  - 궤적 최적화 (LBFGS)
  - 다항식 궤적 생성
- **의존성**: path_optimizer, path_planner, formation_msgs
- **발행**: /V{N}/planning/trajectory
- **구독**: /V{N}/trajectory_command, /V{N}/j_fi/broadcast_traj_recv

### path_manager (라이브러리 + 메시지)
- **역할**:
  - 궤적 계획 라이브러리
  - 메시지 정의 (PolyTraj, PositionCommand)
  - traj_server 노드
- **의존성**: path_optimizer, path_planner
- **라이브러리**: path_manager_lib
- **노드**: traj_server

### path_optimizer (최적화)
- **역할**: LBFGS 궤적 최적화
- **의존성**: path_planner, swarm_graph
- **노드**: 없음 (라이브러리만)

### path_planner (경로 계획)
- **역할**: A* 경로 계획, GridMap
- **의존성**: Eigen3
- **노드**: 없음 (라이브러리만)

### swarm_graph (대형 유사도)
- **역할**: Normalized Laplacian 기반 대형 유사도 계산
- **의존성**: Eigen3
- **노드**: 없음 (라이브러리만)

---

## 🔧 의존성 그래프

```
formation_manager
    ├─ formation_msgs (메시지)
    ├─ swarm_graph (유사도 계산)
    └─ nav_msgs (odometry)

trajectory_planner
    ├─ formation_msgs (메시지)
    ├─ path_manager (라이브러리 + 메시지)
    ├─ path_optimizer (최적화)
    └─ path_planner (A*)

path_manager
    ├─ path_optimizer
    └─ path_planner

path_optimizer
    ├─ path_planner
    └─ swarm_graph

path_planner
    └─ Eigen3

swarm_graph
    └─ Eigen3
```

---

## 📂 파일 구조

```
src/
├── formation_msgs/
│   ├── msg/TrajectoryCommand.msg
│   ├── CMakeLists.txt
│   └── package.xml
│
├── formation_manager/
│   ├── src/
│   │   ├── formation_manager_node.cpp (450줄)
│   │   ├── formation_utils.cpp (149줄)
│   │   └── hungarian_algorithm.cpp (493줄)
│   ├── include/formation_manager/
│   │   ├── formation_utils.h
│   │   └── hungarian_algorithm.h
│   ├── config/scenario_*.yaml
│   ├── CMakeLists.txt
│   └── package.xml
│
├── trajectory_planner/
│   ├── src/
│   │   ├── trajectory_planner_node.cpp (500줄 - 간소화됨)
│   │   ├── path_manager.cpp (873줄)
│   │   ├── polynomial_traj.cpp (224줄)
│   │   └── uniform_bspline.cpp (79줄)
│   ├── include/trajectory_planner/
│   │   ├── path_manager.h
│   │   ├── polynomial_traj.h
│   │   └── uniform_bspline.h
│   ├── config/
│   │   ├── optimizer_params.yaml
│   │   ├── drone_hardware.yaml
│   │   └── map.yaml
│   ├── CMakeLists.txt
│   └── package.xml
│
└── path_manager/ (정리됨)
    ├── src/
    │   ├── path_manager.cpp (873줄)
    │   ├── polynomial_traj.cpp (224줄)
    │   ├── uniform_bspline.cpp (79줄)
    │   └── traj_server.cpp (유지)
    ├── include/path_manager/
    │   ├── path_manager.h
    │   ├── polynomial_traj.h
    │   └── uniform_bspline.h
    ├── msg/
    │   ├── PolyTraj.msg
    │   ├── PositionCommand.msg
    │   ├── FormationTarget.msg (레거시)
    │   └── FormationCommand.msg (레거시)
    ├── CMakeLists.txt
    └── package.xml
```

---

## 🚀 실행 예시

### 전체 시스템
```bash
# Terminal 1: formation_manager
ros2 run formation_manager formation_manager_node \
  --ros-args -p num_drones:=4

# Terminal 2-5: trajectory_planner (각 드론)
ros2 run trajectory_planner trajectory_planner_node \
  --ros-args -p drone_id:=0 -p mavlink_id:=1

ros2 run trajectory_planner trajectory_planner_node \
  --ros-args -p drone_id:=1 -p mavlink_id:=2

# ... (나머지 드론)

# Terminal 6-9: traj_server (각 드론)
ros2 run path_manager traj_server \
  --ros-args -p drone_id:=0

# Terminal 10-13: rover_control (각 드론)
ros2 run rover_control rover_control_node \
  --ros-args -p drone_id:=0
```

### 독립 테스트 (trajectory_planner만)
```bash
# Terminal 1: trajectory_planner
ros2 run trajectory_planner trajectory_planner_node \
  --ros-args -p drone_id:=0 -p rviz_simulation:=true

# Terminal 2: 수동 명령
ros2 topic pub /V1/trajectory_command formation_msgs/msg/TrajectoryCommand \
"{
  drone_id: 0,
  target_position: {x: 10.0, y: 5.0, z: 2.0},
  waypoints: [{x: 10.0, y: 5.0, z: 2.0}]
}"
```

---

## ✅ 핵심 개선사항

| 항목 | Before | After |
|------|--------|-------|
| **패키지 수** | 1 (path_manager) | 3 (분리됨) |
| **코드 라인** | 3,500줄 (혼재) | 각 500줄 수준 |
| **결합도** | 높음 (직접 호출) | 낮음 (토픽 통신) |
| **테스트** | 전체 실행 필요 | 개별 테스트 가능 |
| **대형 관리** | path_manager + FSM | formation_manager만 |
| **궤적 생성** | path_manager + FSM | trajectory_planner만 |
| **SwarmGraph** | 2곳에서 사용 | formation_manager만 |
| **Hungarian** | 2곳에 정의 | formation_manager만 |

---

## 🎯 결론

✅ **완전한 기능 분리**: 대형 관리 ↔ 궤적 생성
✅ **독립 테스트**: 각 모듈을 개별적으로 실행 가능
✅ **토픽 기반**: TrajectoryCommand로 느슨한 결합
✅ **유지보수 개선**: 각 모듈 300~500줄 수준
✅ **확장 용이**: 새로운 대형 알고리즘 쉽게 추가

**핵심 성과**: test node에서 `/V{N}/trajectory_command` 토픽만 발행하면 궤적 생성이 즉시 시작됩니다!
