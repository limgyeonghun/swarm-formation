# 🔧 Swarm Formation 패키지 리팩토링 완료

## 📋 개요

기존 `path_manager` 패키지의 대형 관리 및 궤적 생성 기능을 **토픽 기반 모듈화**로 분리했습니다.

### 목표
- **완전한 기능 분리**: 대형 관리 ↔ 궤적 생성
- **독립 테스트 가능**: 각 모듈을 개별적으로 실행/테스트 가능
- **간소화된 인터페이스**: TrajectoryCommand만 받으면 바로 동작

---

## 📦 새로운 패키지 구조

```
Before (1개 패키지):
path_manager/
├── formation_commander.cpp (789줄) - 대형 관리
├── replan_fsm.cpp (1,806줄) - FSM + 대형 + 궤적 모두 처리
├── path_manager.cpp (873줄) - 궤적 계획
└── formation_utils.cpp, hungarian_algorithm.cpp

After (3개 패키지):
1. formation_msgs/ (메시지 정의)
   └── TrajectoryCommand.msg

2. formation_manager/ (대형 관리 전용)
   ├── formation_manager_node.cpp
   ├── formation_utils.cpp
   ├── hungarian_algorithm.cpp
   └── swarm_graph 의존

3. trajectory_planner/ (궤적 생성 전용)
   ├── trajectory_planner_node.cpp (ReplanFSM 간소화)
   ├── path_manager.cpp
   ├── polynomial_traj.cpp
   └── path_optimizer, path_planner 의존
```

---

## 🔀 데이터 흐름 변경

### **Before (강결합)**
```
FormationCommander
  ↓ (FormationCommand)
ReplanFSM
  ├─ FormationCommand 수신
  ├─ generateFormationPattern() 자체 호출
  ├─ computeHungarianAssignment() 자체 호출
  ├─ publishFormationTarget() → 자기 자신
  ↓
ReplanFSM::formationTargetCallback()
  ├─ 목표점 추출
  ├─ planGlobalTraj()
  └─ 궤적 생성
```

### **After (약결합)**
```
formation_manager
  ├─ YAML 시나리오 로드
  ├─ SwarmGraph 유사도 계산
  ├─ HungarianAlgorithm 실행
  ├─ 각 드론별 개인 목표 계산
  └─ TrajectoryCommand 발행 (드론별)
      ↓ /V{N}/trajectory_command
trajectory_planner
  ├─ TrajectoryCommand 수신
  ├─ 목표점 추출 (이미 계산됨)
  ├─ planGlobalTraj()
  └─ 궤적 생성
```

**핵심 차이:**
- ❌ **제거**: `generateFormationPattern()`, `computeHungarianAssignment()`, `formationCommandCallback()`
- ✅ **간소화**: `trajectoryCommandCallback()` - 목표점만 받아서 바로 처리
- ✅ **테스트 가능**: test node에서 `/V1/trajectory_command` 발행하면 바로 동작

---

## 📨 메시지 정의

### **TrajectoryCommand.msg** (formation_msgs)
```
std_msgs/Header header
int32 drone_id                         # 대상 드론
int32 sequence                         # 미션 시퀀스
string mission_id                      # 미션 ID

geometry_msgs/Point target_position    # 개인 목표점 (이미 할당된)
geometry_msgs/Vector3 target_velocity  # 목표 속도
geometry_msgs/Point[] waypoints        # 경유점들

string formation_type                  # 메타데이터
float64 formation_scale
geometry_msgs/Vector3 formation_offset
```

**중요:** `target_position`과 `waypoints`는 **이미 헝가리안 알고리즘으로 할당된 개인 목표**입니다.

---

## 🚀 빌드 및 실행

### 1. 빌드 순서
```bash
cd /home/lim/workspace/ros_ws/swarm-formation

# 1단계: 메시지 패키지
colcon build --packages-select formation_msgs
source install/setup.bash

# 2단계: 의존 패키지 (순서 중요!)
colcon build --packages-select swarm_graph
colcon build --packages-select path_planner
colcon build --packages-select path_optimizer
source install/setup.bash

# 3단계: 새 패키지
colcon build --packages-select formation_manager
colcon build --packages-select trajectory_planner
source install/setup.bash
```

### 2. 실행 예시

**A. 전체 시스템 (formation_manager + trajectory_planner)**
```bash
# Terminal 1: formation_manager
ros2 run formation_manager formation_manager_node \
  --ros-args -p num_drones:=4

# Terminal 2~5: trajectory_planner (각 드론)
ros2 run trajectory_planner trajectory_planner_node \
  --ros-args -p drone_id:=0 -p mavlink_id:=1

ros2 run trajectory_planner trajectory_planner_node \
  --ros-args -p drone_id:=1 -p mavlink_id:=2

# ... (나머지 드론)
```

**B. 독립 테스트 (trajectory_planner만)**
```bash
# Terminal 1: trajectory_planner
ros2 run trajectory_planner trajectory_planner_node \
  --ros-args -p drone_id:=0

# Terminal 2: 수동 명령 발행
ros2 topic pub /V1/trajectory_command formation_msgs/msg/TrajectoryCommand \
"{
  drone_id: 0,
  sequence: 0,
  mission_id: 'test_mission',
  target_position: {x: 10.0, y: 5.0, z: 2.0},
  waypoints: [
    {x: 5.0, y: 2.5, z: 1.0},
    {x: 10.0, y: 5.0, z: 2.0}
  ],
  formation_type: 'none',
  formation_scale: 1.0
}"
```

---

## ✅ 검증 포인트

### 1. formation_msgs 빌드 확인
```bash
ros2 interface show formation_msgs/msg/TrajectoryCommand
```

### 2. formation_manager 토픽 확인
```bash
# formation_manager 실행 후
ros2 topic list | grep trajectory_command
# 출력 예상:
# /V1/trajectory_command
# /V2/trajectory_command
# /V3/trajectory_command
# /V4/trajectory_command
```

### 3. trajectory_planner 단독 동작 확인
```bash
# trajectory_planner만 실행 후 수동 명령
ros2 topic pub /V1/trajectory_command formation_msgs/msg/TrajectoryCommand \
"{drone_id: 0, target_position: {x: 5.0, y: 0.0, z: 0.0}}"

# 궤적 발행 확인
ros2 topic echo /V1/planning/trajectory
```

---

## 🔧 기존 코드와의 호환성

### path_manager 패키지는 그대로 유지
- 기존 `path_manager` 패키지는 **삭제하지 않음**
- 메시지 정의 (PolyTraj.msg, PositionCommand.msg)는 여전히 사용
- 점진적 마이그레이션 가능

### 마이그레이션 가이드
```bash
# 1. 기존 시스템 유지하면서 테스트
# - formation_manager, trajectory_planner를 별도 namespace에서 실행

# 2. 검증 완료 후 path_manager 비활성화
# - launch 파일에서 formation_commander, replan_fsm 노드 제거

# 3. 최종 정리
# - path_manager에서 formation_commander.cpp, replan_fsm.cpp 삭제 (선택)
```

---

## 📊 코드 변경 요약

| 항목 | Before | After |
|------|--------|-------|
| **패키지 수** | 1 (path_manager) | 3 (formation_msgs, formation_manager, trajectory_planner) |
| **총 라인 수** | ~3,500줄 (혼재) | ~2,000줄 (분리) |
| **결합도** | 높음 (직접 호출) | 낮음 (토픽 통신) |
| **테스트 용이성** | 어려움 (전체 실행 필요) | 쉬움 (개별 테스트 가능) |
| **SwarmGraph 사용** | path_manager + ReplanFSM | formation_manager만 |
| **Hungarian 사용** | FormationCommander + ReplanFSM | formation_manager만 |

---

## 🎯 다음 단계

### 선택 1: 완전 마이그레이션
- `path_manager`에서 formation 관련 코드 완전 제거
- `path_manager`를 순수 궤적 생성 라이브러리로 전환

### 선택 2: 병행 운영
- 기존 `path_manager` 유지 (레거시)
- 새 시스템 점진적 도입

### 선택 3: 추가 최적화
- `trajectory_planner`의 broadcast 궤적 수신 로직 정리
- `formation_manager`의 odometry 기반 위치 추적 개선

---

## 📝 주의사항

1. **namespace 충돌 방지**
   - `path_manager` namespace는 그대로 사용 (PolyTraj 등 메시지 호환성)
   - `formation_manager`, `trajectory_planner` namespace 추가

2. **빌드 순서 엄수**
   - 메시지 → 의존 패키지 → 새 패키지 순서로 빌드

3. **Config 파일 경로**
   - `formation_manager/config/scenario_*.yaml`
   - `trajectory_planner/config/optimizer_params.yaml`

---

## 🐛 트러블슈팅

### 문제 1: "formation_msgs not found"
```bash
# 해결: 메시지 패키지 먼저 빌드
colcon build --packages-select formation_msgs
source install/setup.bash
```

### 문제 2: "SwarmGraph.hpp not found"
```bash
# 해결: swarm_graph 패키지 빌드
colcon build --packages-select swarm_graph
source install/setup.bash
```

### 문제 3: trajectory_planner가 목표를 받지 못함
```bash
# 확인: 토픽 이름 일치 여부
ros2 topic list | grep trajectory_command

# 확인: drone_id 일치 여부
ros2 topic echo /V1/trajectory_command
```

---

## ✨ 결론

✅ **대형 관리**와 **궤적 생성**이 완전히 분리됨
✅ **독립 테스트** 가능 (trajectory_planner만 실행)
✅ **확장 용이** (새로운 대형 알고리즘 쉽게 추가)
✅ **유지보수 개선** (각 모듈 300~500줄 수준)

**핵심 성과:** test node에서 `/V{N}/trajectory_command` 토픽만 발행하면 궤적 생성이 즉시 시작됩니다!
