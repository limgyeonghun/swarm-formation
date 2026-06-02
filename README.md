# Path Manager System

ROS 2 기반 드론/로버 경로 계획 및 편대 제어 시스템

### 시뮬레이션 실행

```bash
ros2 launch path_manager rviz_path_manager.launch.py scenario:=scenario_basic
```

> **참고**:
> - 시작점·목표점은 RViz의 MissionConfig 패널에서 미션 yaml을 로드해 설정합니다
> - `scenario` 파라미터로 다양한 시나리오를 실행할 수 있습니다 (default, risk_zones 등)

### 주요 파라미터

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `scenario` | 시나리오(장애물) 설정 파일 | scenario_basic |
| `drone_id` | 실행할 드론 ID | 1 |
| `world` | ESDF 캐시용 맵 이름 (비우면 optimizer_params 기본값) | (없음) |
| `record_bag` | 궤적 토픽 rosbag 기록 | false |
| `disable_file_logging` | 파일 로깅 비활성화 (콘솔만) | false |