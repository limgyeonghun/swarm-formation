#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog
import os

# 드론별 시작점 오프셋 (상대위치를 절대위치로 변환)
DRONE_OFFSETS = {
    'ugv1': {'x': 0.0, 'y': 0.0, 'z': 0.0},      # drone_0
    'ugv2': {'x': 0.24, 'y': 1.99, 'z': 0.0},    # drone_1
    'ugv3': {'x': 0.49, 'y': 3.97, 'z': 0.0},    # drone_2
    'ugv4': {'x': 0.73, 'y': 5.96, 'z': 0.0}     # drone_3
}

# 장애물 위치
OBSTACLES = [
    [-1.81, -13.04, 0.0],   # 1
    [-1.29, -17.08, 0.0],   # 2
    [-3.54, -23.30, 0.0],   # 3
    [-5.46, -29.95, 0.0],   # 4
    [-6.20, -42.81, 0.0],   # 5
    [-7.60, -52.01, 0.0],   # 6
    [27.90, -75.40, 0.0],   # 7
    [32.80, -87.10, 0.0],   # 8
    [29.20, -112.50, 0.0],  # 9
    [32.80, -129.00, 0.0],  # 10
    [50.99, -130.57, 0.0],  # 11
    [68.56, -133.24, 0.0]   # 13
]

def extract_position_data(ulog_path):
    """ULog 파일에서 위치 데이터 추출"""
    try:
        ulog = ULog(ulog_path)

        # vehicle_local_position 토픽에서 위치 데이터 추출
        position_data = None
        for data in ulog.data_list:
            if 'vehicle_local_position' in data.name:
                position_data = data
                break

        if position_data is None:
            print(f"Warning: vehicle_local_position not found in {ulog_path}")
            return None, None, None

        # x, y 위치 추출
        x = position_data.data['x']
        y = position_data.data['y']
        timestamps = position_data.data['timestamp']

        return x, y, timestamps

    except Exception as e:
        print(f"Error reading {ulog_path}: {e}")
        return None, None, None

def find_position_at_time(timestamps, x_abs, y_abs, target_time):
    """특정 시간에 가장 가까운 위치 찾기"""
    if timestamps is None or len(timestamps) == 0:
        return None, None

    # 타임스탬프를 초 단위로 변환 (마이크로초 -> 초)
    timestamps_sec = timestamps / 1e6

    # 가장 가까운 인덱스 찾기
    idx = np.argmin(np.abs(timestamps_sec - target_time))
    return x_abs[idx], y_abs[idx]


def plot_trajectories(log_dir, snapshot_times=None):
    """
    궤적 그래프 생성 - 논문용 스타일 (단일 그래프)

    Args:
        log_dir: 로그 디렉토리
        snapshot_times: 특정 시점 리스트 (상대 시간, 초 단위). 예: [30, 60, 90]
    """
    _, ax = plt.subplots(figsize=(10, 12))

    # 각 드론의 궤적 그리기
    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e']  # 더 선명한 색상
    drone_names = ['ugv1', 'ugv2', 'ugv3', 'ugv4']

    # 모든 드론의 데이터를 저장
    all_drone_data = {}

    for idx, drone_name in enumerate(drone_names):
        ulog_path = os.path.join(log_dir, f"{drone_name}.ulg")

        if not os.path.exists(ulog_path):
            print(f"File not found: {ulog_path}")
            continue

        print(f"Processing {drone_name}...")
        x, y, timestamps = extract_position_data(ulog_path)

        if x is None or y is None:
            continue

        # 오프셋 적용 (상대위치 -> 절대위치)
        offset = DRONE_OFFSETS[drone_name]
        y_abs = x + offset['x']
        x_abs = y + offset['y']

        # 데이터 저장
        all_drone_data[drone_name] = {
            'x_abs': x_abs,
            'y_abs': y_abs,
            'timestamps': timestamps,
            'idx': idx,
            'color': colors[idx]
        }

        # 궤적 그리기
        ax.plot(x_abs, y_abs, color=colors[idx], linewidth=2,
                label=f'UGV {idx+1}', alpha=0.85, zorder=3)

        # 시작점 표시 (원)
        ax.scatter(x_abs[0], y_abs[0], color=colors[idx], s=20,
                   marker='o', linewidths=1.5, zorder=5)

        # 종료점 표시 (사각형)
        ax.scatter(x_abs[-1], y_abs[-1], color=colors[idx], s=20,
                   marker='s', linewidths=1.5, zorder=5)

        print(f"  Start: ({x_abs[0]:.2f}, {y_abs[0]:.2f})")
        print(f"  End: ({x_abs[-1]:.2f}, {y_abs[-1]:.2f})")
        print(f"  Points: {len(x)}")

    # 특정 시점의 위치 표시
    if snapshot_times and all_drone_data:
        print(f"\nMarking snapshot times: {snapshot_times}")

        # 각 드론의 시작 시간 찾기 (동기화 기준)
        start_times = {}
        for drone_name, data in all_drone_data.items():
            start_times[drone_name] = data['timestamps'][0] / 1e6

        # 가장 늦게 시작한 드론을 기준으로 동기화
        reference_start_time = max(start_times.values())
        print(f"Reference start time: {reference_start_time:.2f}s")

        for snap_time in snapshot_times:
            absolute_time = reference_start_time + snap_time
            print(f"\n  Snapshot at t={snap_time}s (absolute: {absolute_time:.2f}s):")

            snapshot_positions = []
            for drone_name, data in all_drone_data.items():
                x_pos, y_pos = find_position_at_time(
                    data['timestamps'],
                    data['x_abs'],
                    data['y_abs'],
                    absolute_time
                )

                if x_pos is not None and y_pos is not None:
                    snapshot_positions.append((x_pos, y_pos, data['color']))
                    print(f"    {drone_name}: ({x_pos:.2f}, {y_pos:.2f})")

            # 스냅샷 위치 표시 (다이아몬드 마커, 크게)
            for x_pos, y_pos, color in snapshot_positions:
                ax.scatter(x_pos, y_pos, color=color, s=15,
                          marker='D', linewidths=2.5,
                          zorder=6, alpha=0.9)



    # 장애물 그리기 - 작고 깔끔하게
    obstacles_y = [obs[0] for obs in OBSTACLES]
    obstacles_x = [obs[1] for obs in OBSTACLES]

    ax.scatter(obstacles_x, obstacles_y, color='black', s=50, alpha=0.7,
               marker='o', edgecolors='black', linewidths=2,
               label='Obstacles', zorder=4)

    # 축 설정 (제목 제거, 폰트 크기 증가)
    ax.set_xlabel('X Position (m)', fontsize=20, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=20, fontweight='bold')

    # 범례 설정
    ax.legend(loc='upper right', fontsize=17, framealpha=0.95,
              edgecolor='black')

    # 격자 설정
    ax.grid(True, alpha=0.25, linestyle='--', linewidth=0.5)
    ax.set_aspect('equal', adjustable='box')

    # 틱 라벨 크기 설정
    ax.tick_params(axis='both', labelsize=15)

    # 여백 조정
    plt.tight_layout()

    # 저장
    output_path = os.path.join(log_dir, 'trajectories_paper_style.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"\nPlot saved to: {output_path}")

    plt.show()

if __name__ == '__main__':
    log_directory = '/home/lim/workspace/ros_ws/swarm-formation/logs/px4/1215'

    # 특정 시점 설정 (초 단위, 상대 시간)
    # 예: [30, 60, 90] = 시작 후 30초, 60초, 90초
    snapshot_times = [83,103,120,145,161,179,205]  # 원하는 시점으로 수정 가능

    print("Plotting PX4 ULog trajectories...")
    print(f"Log directory: {log_directory}\n")

    plot_trajectories(log_directory, snapshot_times=snapshot_times)

    print("\nDone!")
