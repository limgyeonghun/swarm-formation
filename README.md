# Path Manager System

ROS 2 기반 드론/로버 경로 계획 및 편대 제어 시스템

### 시뮬레이션 실행

**터미널 1** - 메인 시스템:
```bash
ros2 launch path_manager rviz_path_manager.launch.py scenario:=scenario_basic
```

**터미널 2** - 편대 명령 트리거 (선택사항):
```bash
ros2 run formation_manager formation_manager_node \
  --ros-args -p num_drones:=1 -p scenario:=risk_zones
```

> **참고**:
> - 터미널 2의 formation_manager가 시작점과 목표점을 설정합니다
> - `scenario` 파라미터로 다양한 시나리오를 실행할 수 있습니다 (default, risk_zones 등)

### 주요 파라미터

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `real` | 실제 하드웨어 모드 활성화 | false |
| `rviz_simulation` | RViz 시각화 활성화 | false |
| `enable_visualization` | 경로 시각화 노드 활성화 | false |
| `map_config` | 맵 설정 파일 (map, map_risk_zones) | map |
| `risk_zones` | 위협 지역 설정 파일 | (없음) |
| `drone_id` | 실행할 드론 ID (실제 하드웨어 모드) | 1 |