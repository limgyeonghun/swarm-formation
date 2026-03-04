# 🧹 코드 정리 가이드

## 📋 현재 상태

### 새로 생성된 패키지 (사용할 것)
```
✅ formation_msgs/        - 메시지 정의 (TrajectoryCommand)
✅ formation_manager/     - 대형 관리
✅ trajectory_planner/    - 궤적 생성
```

### 기존 패키지 (정리 필요)
```
⚠️ path_manager/          - 이전 코드 + 메시지 정의 혼재
   ├── formation_commander.cpp  ❌ 삭제 예정 (formation_manager로 이동함)
   ├── replan_fsm.cpp          ❌ 삭제 예정 (trajectory_planner로 이동함)
   ├── formation_utils.cpp     ❌ 삭제 예정 (중복)
   ├── hungarian_algorithm.cpp ❌ 삭제 예정 (중복)
   ├── path_manager.cpp        ✅ 유지 (라이브러리)
   ├── polynomial_traj.cpp     ✅ 유지 (라이브러리)
   ├── uniform_bspline.cpp     ✅ 유지 (라이브러리)
   ├── traj_server.cpp         ✅ 유지
   └── msg/                    ✅ 유지 (PolyTraj, PositionCommand)
```

---

## 🎯 정리 계획

### Option 1: 최소 변경 (권장)
**path_manager를 메시지 + 라이브러리 패키지로 유지**

```bash
# path_manager에서 삭제할 파일
rm src/path_manager/src/formation_commander.cpp
rm src/path_manager/src/replan_fsm.cpp
rm src/path_manager/src/formation_utils.cpp
rm src/path_manager/src/hungarian_algorithm.cpp
rm src/path_manager/include/path_manager/replan_fsm.h
rm src/path_manager/include/path_manager/formation_utils.h
rm src/path_manager/include/path_manager/hungarian_algorithm.h

# CMakeLists.txt에서 제거
# - add_executable(formation_commander ...)
# - add_executable(replan_fsm_node ...)
# - formation_utils.cpp, hungarian_algorithm.cpp 제거

# 유지되는 것
# - path_manager.cpp (라이브러리)
# - polynomial_traj.cpp (라이브러리)
# - uniform_bspline.cpp (라이브러리)
# - traj_server.cpp (노드)
# - msg/ (메시지 정의)
```

**장점:**
- 기존 의존성 유지 (다른 패키지 수정 불필요)
- PolyTraj, PositionCommand 메시지 그대로 사용
- path_manager는 순수 라이브러리 + 메시지 패키지로 전환

**단점:**
- path_manager 이름이 약간 혼란스러울 수 있음

---

### Option 2: 완전 분리 (복잡함)
**path_manager를 3개 패키지로 완전 분리**

```
path_manager → 3개 분리
├── path_msgs/           (메시지만)
│   ├── PolyTraj.msg
│   └── PositionCommand.msg
├── path_planning_lib/   (라이브러리만)
│   ├── path_manager.cpp
│   ├── polynomial_traj.cpp
│   └── uniform_bspline.cpp
└── traj_server/         (노드)
    └── traj_server.cpp
```

**장점:**
- 완전히 깔끔한 구조
- 각 패키지의 역할 명확

**단점:**
- 모든 패키지의 의존성 수정 필요
- path_optimizer, trajectory_planner 등 모두 수정
- 빌드 순서 복잡

---

## ✅ 추천: Option 1 실행

### Step 1: 백업
```bash
cd /home/lim/workspace/ros_ws/swarm-formation
git add -A
git commit -m "backup: before cleanup"
```

### Step 2: path_manager 정리
```bash
# 불필요한 파일 삭제
rm src/path_manager/src/formation_commander.cpp
rm src/path_manager/src/replan_fsm.cpp
rm src/path_manager/src/formation_utils.cpp
rm src/path_manager/src/hungarian_algorithm.cpp
rm src/path_manager/include/path_manager/replan_fsm.h
rm src/path_manager/include/path_manager/formation_utils.h
rm src/path_manager/include/path_manager/hungarian_algorithm.h

# Config 파일 정리 (중복 제거)
rm src/path_manager/config/scenario_*.yaml
# (scenario 파일은 formation_manager에만 있으면 됨)
```

### Step 3: path_manager CMakeLists.txt 수정
```cmake
# path_manager/CMakeLists.txt

# ❌ 제거할 부분:
# add_executable(formation_commander ...)
# add_executable(replan_fsm_node ...)
# formation_utils.cpp, hungarian_algorithm.cpp

# ✅ 유지할 부분:
add_library(${PROJECT_NAME}_lib
  src/path_manager.cpp
  src/polynomial_traj.cpp
  src/uniform_bspline.cpp
  ../common/log_manager.cpp
)

add_executable(traj_server
  src/traj_server.cpp
)

# 메시지 생성 유지
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolyTraj.msg"
  "msg/FormationTarget.msg"
  "msg/FormationCommand.msg"
  "msg/PositionCommand.msg"
  DEPENDENCIES std_msgs builtin_interfaces nav_msgs geometry_msgs
)
```

### Step 4: 빌드 테스트
```bash
cd /home/lim/workspace/ros_ws/swarm-formation

# 1. path_manager 빌드
colcon build --packages-select path_manager
source install/setup.bash

# 2. 새 패키지 빌드
colcon build --packages-select formation_manager trajectory_planner
source install/setup.bash

# 3. 전체 빌드
colcon build
```

---

## 📊 정리 후 최종 구조

```
src/
├── formation_msgs/              # 새 메시지
│   └── msg/TrajectoryCommand.msg
│
├── formation_manager/           # 대형 관리
│   ├── src/formation_manager_node.cpp
│   ├── src/formation_utils.cpp
│   ├── src/hungarian_algorithm.cpp
│   └── config/scenario_*.yaml
│
├── trajectory_planner/          # 궤적 생성
│   ├── src/trajectory_planner_node.cpp
│   ├── src/path_manager.cpp (복사본)
│   ├── src/polynomial_traj.cpp (복사본)
│   └── src/uniform_bspline.cpp (복사본)
│
├── path_manager/                # 라이브러리 + 메시지 (정리됨)
│   ├── src/path_manager.cpp     ✅ 원본
│   ├── src/polynomial_traj.cpp  ✅ 원본
│   ├── src/uniform_bspline.cpp  ✅ 원본
│   ├── src/traj_server.cpp      ✅ 유지
│   └── msg/                     ✅ 기존 메시지 유지
│       ├── PolyTraj.msg
│       ├── PositionCommand.msg
│       ├── FormationTarget.msg
│       └── FormationCommand.msg
│
├── path_optimizer/              # 유지
├── path_planner/                # 유지
├── swarm_graph/                 # 유지
└── ... (다른 패키지)
```

---

## 🚨 주의사항

### 1. 메시지 호환성
- `path_manager/msg/PolyTraj`는 여러 패키지에서 사용 중
- 삭제하면 안 됨!
- trajectory_planner도 `path_manager::msg::PolyTraj` 사용

### 2. namespace 유지
- `path_manager` namespace는 유지 (기존 코드 호환성)
- 새 패키지는 `formation_manager`, `trajectory_planner` namespace

### 3. 의존성 체크
```bash
# path_manager를 참조하는 패키지 확인
grep -r "path_manager" src/*/CMakeLists.txt
grep -r "path_manager" src/*/package.xml
```

---

## ✨ 정리 완료 후 확인사항

### 1. 빌드 성공 확인
```bash
colcon build
# 모든 패키지 빌드 성공해야 함
```

### 2. 노드 실행 확인
```bash
# formation_manager
ros2 run formation_manager formation_manager_node

# trajectory_planner
ros2 run trajectory_planner trajectory_planner_node

# traj_server (path_manager)
ros2 run path_manager traj_server
```

### 3. 토픽 확인
```bash
ros2 topic list | grep -E "(trajectory_command|planning/trajectory)"
```

---

## 🎯 최종 목표

✅ **path_manager = 라이브러리 + 메시지 패키지**
- formation 관련 노드 제거
- 순수 궤적 계산 라이브러리로 유지
- 메시지 정의 유지 (하위 호환성)

✅ **formation_manager = 대형 관리 전용**
✅ **trajectory_planner = 궤적 생성 전용**

이렇게 하면 기존 코드 호환성을 유지하면서도 깔끔하게 분리됩니다!
