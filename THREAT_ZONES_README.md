# Threat Zone System - 방공망 파훼 경로 계획

방공망(SAM, AAA, Radar)을 통과하는 최적 경로 생성 시스템

## 개요

단순한 0/1 장애물이 아닌 **그래디언트 기반 위협도 필드**를 사용:
- 모든 구역이 방공망으로 막혀있어도 **가장 위협도가 낮은 경로**로 돌파
- 여러 방공망이 겹칠수록 위협도 누적 (가우시안 + 선형 감쇠)
- RViz에서 **3D 색상 그래디언트**로 시각화 (파란색=안전, 빨간색=위험)
- **3D 위협 계산**: Z축(고도) 포함하여 구형(sphere) 범위로 위협도 계산

## 설정 파일 구조

### 1. `scenario_threat_zones.yaml` - 미션 시나리오
드론 시작 위치 + 웨이포인트 정의

```yaml
/**:
  ros__parameters:
    drone_0:
      start_point_x: 0.0
      start_point_y: 0.0
      start_point_z: 0.0

    # ... 다른 드론들 ...

    mission:
      commands:
        - formation_type: "line_first"
          formation_scale: 2.0
          distance_threshold: 5.0
          waypoints:
            - [10.0, 0.0, 0.0]
            - [35.0, 0.0, 0.0]
            - [60.0, 0.0, 0.0]
```

### 2. `threat_zones.yaml` - 방공망 정의 (새로운 파일)
방공망 위치, 범위, 위협도 정의

```yaml
/**:
  ros__parameters:
    threat_zones: [
      # SAM Site 1
      25.0, 0.0, 0.0,    # center (x, y, z)
      15.0,              # detection range (m)
      8.0,               # engagement range (m)
      100.0,             # max threat level

      # SAM Site 2
      40.0, -10.0, 0.0,
      15.0, 8.0, 100.0,

      # AAA Site
      30.0, 15.0, 0.0,
      12.0, 6.0, 50.0,

      # Radar Site
      50.0, 5.0, 0.0,
      25.0, 10.0, 30.0
    ]
```

### 3. `map_threat_zones.yaml` - 맵 설정 (threat zone 전용)
위협 존 시나리오 전용 맵 설정

```yaml
/**:
  ros__parameters:
    grid_map/resolution: 0.1
    grid_map/map_size_x: 80.0
    grid_map/map_size_y: 60.0
    grid_map/map_size_z: 30.0          # 3D threat visualization (0~30m)
    grid_map/map_origin_x: -10.0
    grid_map/map_origin_y: -30.0

    # IMPORTANT: Enable threat zones
    grid_map/use_threat_zones: true
    grid_map/threat_cost_weight: 1.0

    # 3D Visualization settings
    enable_threat_zones: true
    threat_visualization_resolution: 3.5  # Larger = less cubes, better visibility
```

### 4. `optimizer_params.yaml` - 최적화 파라미터
위협도 가중치 포함

```yaml
/**:
  ros__parameters:
    optimization/weight_obstacle: 50000.0   # Hard constraints
    optimization/weight_threat: 100.0       # Soft constraints (NEW!)
    optimization/weight_swarm: 50000.0
    # ... 나머지 파라미터 ...
```

### 5. `obstacles.yaml` - 하드 장애물 (선택적)
물리적 장애물 (건물, 지형 등)

```yaml
/**:
  ros__parameters:
    obstacles: [
      # 필요시 추가
    ]
```

## 실행 방법

### 1. 빌드
```bash
# Docker 컨테이너 진입 (호스트에서)
docker exec -it rover_nvidia bash

# ROS2 빌드 (컨테이너 내부)
cd /workspace/ros_ws/swarm-formation
colcon build --packages-select path_planner path_optimizer path_visualization path_manager
source install/setup.bash
```

### 2. 실행
```bash
ros2 launch path_manager path_manager.launch.py \
  scenario:=scenario_threat_zones \
  map_config:=map_threat_zones \
  threat_zones:=threat_zones
```

**주요 파라미터:**
- `scenario`: 미션 시나리오 파일 (드론 시작 위치 + 웨이포인트)
- `map_config`: 맵 설정 파일 (use_threat_zones=true 포함)
- `threat_zones`: 방공망 정의 파일

### 3. RViz 확인

RViz에서 다음 토픽 활성화:
- `/threat_field` (Marker) - **3D 방공망 메쉬 그라데이션 시각화**
  - 각 방공망을 **삼각형 메쉬로 구성된 구형 표면**으로 표현 (TRIANGLE_LIST)
  - 각 꼭짓점의 색상이 **실제 위협도를 반영**하여 면에 부드러운 그라데이션 생성:
    - **중심 근처**: 빨강 (위협도 100%) - 불투명
    - **중간 거리**: 노랑/초록 (위협도 감소) - 반투명
    - **외곽 경계**: 파랑 (위협도 낮음) - 매우 투명
  - **연속적인 면**: 점이 아닌 실제 면으로 표현되어 원형 표면 시각화
  - **겹침 효과**: 방공망이 겹치면 각 면이 합산된 위협도로 표시
- `/opt_trajectory` (Marker) - 최적화된 궤적
- `/simple_path_trajectory` (Marker) - A* 초기 경로

**확인 사항:**
- 경로가 빨간색 영역을 피하고 파란색/초록색 영역으로 우회하는지
- 방공망 겹침 구역(노란색/빨간색)을 최소화하는지
- **3D 뷰**: RViz에서 카메라 각도를 회전하여 고도별 위협 분포 확인

## 가중치 조정 가이드

### `optimization/weight_threat` (optimizer_params.yaml)
위협 존 회피 강도 조절

```yaml
optimization/weight_threat: 100.0   # 기본값
```

**조정 효과:**
- `50~100`: 위협 존을 약하게 회피 (직선에 가까운 경로)
- `100~300`: 균형잡힌 회피 (권장)
- `300~1000`: 적극적인 회피 (경로가 크게 우회)
- `>1000`: 거의 절대 회피 (하드 제약처럼 동작)

**주의:** `weight_obstacle`(50000)보다 훨씬 낮게 유지해야 소프트 제약으로 동작

### `grid_map/threat_cost_weight` (map_threat_zones.yaml)
A* 경로 계획에서 위협도 비용 배수

```yaml
grid_map/threat_cost_weight: 1.0   # 기본값
```

**조정 효과:**
- `0.5`: A* 초기 경로가 위협 존 덜 회피 → Optimizer가 더 수정
- `1.0`: 균형 (권장)
- `2.0`: A* 초기 경로가 위협 존 적극 회피 → 최적화 부담 감소

## 위협 존 타입별 권장 값

### SAM (Surface-to-Air Missile)
```yaml
detection_range: 15.0      # 탐지 범위
engagement_range: 8.0      # 교전 범위
max_threat_level: 100.0    # 최고 위협
```

### AAA (Anti-Aircraft Artillery)
```yaml
detection_range: 12.0
engagement_range: 6.0
max_threat_level: 50.0     # 중간 위협
```

### Radar (탐지 전용)
```yaml
detection_range: 25.0      # 넓은 범위
engagement_range: 10.0
max_threat_level: 30.0     # 낮은 위협
```

## 위협도 계산 방식

### Engagement 범위 내 (고위협)
```
위협도 = max_threat_level × exp(-0.5 × (dist/σ)²)
σ = engagement_range / 3.0
```
- 가우시안 감쇠
- 중심부가 가장 위험
- 부드러운 전환

### Detection 범위 내 (중위협)
```
위협도 = max_threat_level × 0.3 × (1 - ratio)
ratio = (dist - engagement) / (detection - engagement)
```
- 선형 감쇠
- 30% 위협도로 시작
- 범위 끝에서 0

### 범위 밖
```
위협도 = 0
```

### 여러 방공망 겹침
```
총 위협도 = Σ (각 방공망의 위협도)
```
- 단순 합산 (겹칠수록 위험)

## 문제 해결

### Q: 위협도 필드가 RViz에 안 보여요
**A:**
1. `map_threat_zones.yaml`에서 `use_threat_zones: true` 확인
2. `threat_zones.yaml` 로딩 확인 (launch 파라미터)
3. `/threat_field` 토픽 활성화 확인

### Q: 경로가 방공망을 완전히 무시해요
**A:**
- `optimization/weight_threat` 값을 높이세요 (100→300)
- `grid_map/threat_cost_weight` 값을 높이세요 (1.0→2.0)

### Q: 경로가 너무 크게 우회해요
**A:**
- `optimization/weight_threat` 값을 낮추세요 (300→100)
- 또는 `max_threat_level` 값을 낮추세요 (100→50)

### Q: 방공망 겹침이 제대로 반영 안 돼요
**A:**
- `getThreatLevel()` 함수가 모든 방공망을 합산하는지 확인
- RViz에서 노란색/빨간색 영역이 겹침 부분에 생기는지 확인

### Q: 성능이 느리거나 시각화가 너무 압도적이에요
**A:**
- 삼각형 메쉬 방식 사용 (각 방공망당 ~2500 삼각형)
- 코드에서 메쉬 밀도 조정 ([path_visualization.cpp:796-798](src/path_visualization/src/path_visualization.cpp#L796-L798)):
  - `num_radial_layers`: 5 → 3 (동심원 레이어 수)
  - `num_latitude`: 16 → 12 (위도 분할)
  - `num_longitude`: 32 → 24 (경도 분할)
- 임계값 조정: `avg_threat < 1.0` → `< 5.0` (낮은 위협도 면 숨김)

## 코드 구조

### GridMap (src/path_planner/)
- `ThreatZone` 구조체: 방공망 정의
- `getThreatLevel()`: 위치별 위협도 (가우시안)
- `getThreatGradient()`: 위협도 그래디언트 (최적화용)
- `updateThreatField()`: 전체 맵 사전 계산

### AStar (src/path_planner/)
- `getThreatCost()`: A* 비용에 위협도 추가
- g-score에 threat_cost 반영

### PolyTrajOptimizer (src/path_optimizer/)
- `threatGradCostP()`: 제곱 비용 함수
- `cost = weight_threat × threat²`
- 그래디언트 기반 회피

### PathVisualization (src/path_visualization/)
- `publishThreatField()`: CUBE_LIST로 시각화
- 색상 그래디언트 (Blue→Green→Yellow→Red)

## 예상 동작

**시나리오:** 4개 방공망이 직선 경로 차단
```
Start(0,0) ---[SAM1]---[AAA]---[SAM2]---[Radar]--- Goal(60,0)
```

**결과:**
1. A* 경로: 방공망 겹침이 적은 측면으로 초기 우회
2. 궤적 최적화: 위협도 그래디언트를 따라 부드럽게 조정
3. RViz: 파란색/초록색 영역 선호, 빨간색 영역 최소화

## 향후 개선

- [ ] 타원형 방공망 (수평/수직 다른 범위)
- [ ] 방공망 타입별 다른 감쇠 함수
- [ ] 시간대별 위협도 변화 (레이더 회전)
- [ ] 3D 위협도 (고도별 차등)
- [ ] 실시간 위협도 업데이트 (동적 방공망)
