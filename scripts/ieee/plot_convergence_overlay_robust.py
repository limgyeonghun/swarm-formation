#!/usr/bin/env python3
"""
논문용 수렴 곡선 오버레이 그래프 (이상치 제거 버전)
Robust statistics: 상위 10% 이상치를 제거한 데이터만 사용
"""

import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from pathlib import Path
from datetime import datetime
from scipy.spatial.distance import pdist, squareform
from scipy.optimize import linear_sum_assignment


class ShapeAnalyzer:
    """Shape-based formation similarity analyzer"""

    @staticmethod
    def normalize_shape(positions):
        centered = positions - np.mean(positions, axis=0)
        distances = pdist(centered, metric='euclidean')
        dist_matrix = squareform(distances)
        char_length = np.mean(distances[distances > 0])
        if char_length < 1e-6:
            return dist_matrix
        return dist_matrix / char_length

    @staticmethod
    def shape_similarity(actual_pos, target_pos):
        if len(actual_pos) != len(target_pos):
            return 1.0
        actual_norm = ShapeAnalyzer.normalize_shape(actual_pos)
        target_norm = ShapeAnalyzer.normalize_shape(target_pos)
        n = len(actual_pos)
        cost_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                cost_matrix[i, j] = np.sum(np.abs(actual_norm[i, :] - target_norm[j, :]))
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        permuted_actual = actual_norm[row_ind, :][:, row_ind]
        permuted_target = target_norm[col_ind, :][:, col_ind]
        diff = permuted_actual - permuted_target
        return np.sqrt(np.mean(diff**2))


def parse_timestamp(log_line):
    match = re.search(r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]', log_line)
    if match:
        try:
            return datetime.strptime(match.group(1), '%Y-%m-%d %H:%M:%S.%f')
        except:
            pass
    return None


def extract_target_formation(log_file):
    formations = []
    with open(log_file, 'r') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i]
        if 'Formation command:' in line:
            ts = parse_timestamp(line)
            formation_type_match = re.search(r'Formation command: (\w+)', line)
            if formation_type_match and ts:
                formation_type = formation_type_match.group(1)
                for k in range(i + 1, min(i + 50, len(lines))):
                    if 'Formation Positions:' in lines[k]:
                        pattern = []
                        j = k + 1
                        while j < len(lines) and j < k + 20:
                            pos_match = re.search(r'Drone \d+:\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]', lines[j])
                            if pos_match:
                                x, y = float(pos_match.group(1)), float(pos_match.group(2))
                                pattern.append([x, y])
                            elif '=====' in lines[j] or 'Formation' in lines[j]:
                                break
                            j += 1
                        if pattern:
                            formations.append({
                                'time': ts,
                                'type': formation_type,
                                'positions': np.array(pattern)
                            })
                            i = j
                            break
        i += 1
    return formations


def extract_actual_positions(log_file):
    positions = []
    with open(log_file, 'r') as f:
        for line in f:
            if 'Start:' in line:
                ts = parse_timestamp(line)
                pos_match = re.search(r'Start: \(([-\d.]+),([-\d.]+),([-\d.]+)\)', line)
                if ts and pos_match:
                    x, y = float(pos_match.group(1)), float(pos_match.group(2))
                    positions.append({'time': ts, 'position': [x, y]})
    return positions


def analyze_single_test(test_dir, num_drones):
    test_dir = Path(test_dir)
    if not test_dir.exists():
        return None

    drone_logs = []
    for drone_id in range(num_drones):
        log_files = list(test_dir.glob(f"replan_fsm_drone_{drone_id}_*.log"))
        if log_files:
            drone_logs.append((drone_id, log_files[0]))

    if not drone_logs:
        return None

    formations = extract_target_formation(drone_logs[0][1])
    if not formations:
        return None

    all_positions = {}
    for drone_id, log_file in drone_logs:
        positions = extract_actual_positions(log_file)
        all_positions[drone_id] = positions

    mission_start = formations[0]['time']
    time_series = []
    all_timestamps = set()

    for positions_list in all_positions.values():
        for pos_data in positions_list:
            all_timestamps.add(pos_data['time'])

    sorted_timestamps = sorted(all_timestamps)
    current_formation_idx = 0

    for ts in sorted_timestamps:
        elapsed_time = (ts - mission_start).total_seconds()
        if elapsed_time < 0:
            continue

        for i in range(len(formations) - 1, -1, -1):
            if (formations[i]['time'] - mission_start).total_seconds() <= elapsed_time:
                current_formation_idx = i
                break

        target_formation = formations[current_formation_idx]
        target_positions = target_formation['positions']

        actual_positions = []
        for drone_id in range(num_drones):
            if drone_id in all_positions:
                drone_positions = all_positions[drone_id]
                closest_pos = min(drone_positions,
                                key=lambda p: abs((p['time'] - ts).total_seconds()),
                                default=None)
                if closest_pos and abs((closest_pos['time'] - ts).total_seconds()) < 0.5:
                    actual_positions.append(closest_pos['position'])

        if len(actual_positions) == num_drones and len(target_positions) == num_drones:
            actual_pos_array = np.array(actual_positions)
            actual_centered = actual_pos_array - np.mean(actual_pos_array, axis=0)
            target_centered = target_positions - np.mean(target_positions, axis=0)
            similarity = ShapeAnalyzer.shape_similarity(actual_centered, target_centered)
            time_series.append({
                'time': elapsed_time,
                'similarity': similarity,
                'formation': target_formation['type']
            })

    return time_series


def moving_average(data, window_size):
    if len(data) < window_size:
        return np.array(data)
    kernel = np.ones(window_size) / window_size
    smoothed = np.convolve(data, kernel, mode='valid')
    padding = np.full(window_size - 1, smoothed[0])
    return np.concatenate([padding, smoothed])


def filter_outlier_tests(all_tests_data):
    """
    이상치 테스트를 필터링
    각 테스트의 최종 평균 similarity를 기준으로 상위 10% 제거
    """
    if not all_tests_data:
        return []

    # 각 테스트의 평균 similarity 계산
    test_scores = []
    for i, data in enumerate(all_tests_data):
        similarities = np.array(data['similarities'])
        # 후반부 (안정 상태) 평균 사용
        stable_region = similarities[len(similarities)//2:]
        if len(stable_region) > 0:
            avg_similarity = np.mean(stable_region)
        else:
            avg_similarity = np.mean(similarities)
        test_scores.append({'index': i, 'score': avg_similarity, 'data': data})

    # 점수 기준 정렬
    test_scores.sort(key=lambda x: x['score'])

    # 상위 10% 제거
    n_keep = int(len(test_scores) * 0.9)
    if n_keep < 1:
        n_keep = len(test_scores)

    filtered_data = [item['data'] for item in test_scores[:n_keep]]

    print(f"    총 {len(all_tests_data)}개 테스트 중 {len(filtered_data)}개 사용 (상위 {len(all_tests_data)-len(filtered_data)}개 제외)")

    return filtered_data


def plot_convergence_overlay_robust():
    """이상치 제거한 robust 오버레이 그래프"""

    # Publication-quality settings
    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 15,
        'axes.labelsize': 20,
        'axes.titlesize': 22,
        'xtick.labelsize': 17,
        'ytick.labelsize': 17,
        'legend.fontsize': 18,
        'axes.linewidth': 1.5,
        'grid.linewidth': 0.8,
        'lines.linewidth': 3.0,
    })

    log_dir = Path("/home/lim/workspace/ros_ws/swarm-formation/logs/runtime/multi_rover")
    output_dir = Path("/home/lim/workspace/ros_ws/swarm-formation/results/scalability")

    drone_counts = [4, 6, 8, 10]

    # Professional color scheme
    colors = {
        4: '#0173B2',   # Blue
        6: '#DE8F05',   # Orange
        8: '#029E73',   # Green
        10: '#CC78BC'   # Purple
    }

    fig, ax = plt.subplots(figsize=(14, 9))

    print("\n" + "="*80)
    print("논문용 수렴 곡선 오버레이 생성 (Robust - 이상치 제거)")
    print("="*80)

    max_time = 0
    robust_stats = {}

    for num_drones in drone_counts:
        print(f"\n처리 중: {num_drones}대 드론")

        all_tests_data = []

        for test_num in range(1, 11):
            test_dir = log_dir / f"{num_drones}rover" / f"test{test_num}"
            time_series = analyze_single_test(test_dir, num_drones)

            if time_series:
                times = [d['time'] for d in time_series]
                similarities = [d['similarity'] for d in time_series]

                if len(similarities) > 10:
                    similarities = moving_average(similarities, window_size=5)

                all_tests_data.append({'times': times, 'similarities': similarities})

        # 이상치 필터링
        filtered_data = filter_outlier_tests(all_tests_data)

        if not filtered_data:
            print(f"  경고: {num_drones}대 드론 데이터 없음")
            continue

        # 시간 축 정렬 및 평균 계산
        all_times = []
        for data in filtered_data:
            all_times.extend(data['times'])

        if all_times:
            local_max_time = max(all_times)
            max_time = max(max_time, local_max_time)

            time_grid = np.linspace(0, local_max_time, 1000)
            interpolated = []

            for data in filtered_data:
                times = np.array(data['times'])
                similarities = np.array(data['similarities'])
                if len(times) > 1:
                    interp_sim = np.interp(time_grid, times, similarities)
                    interpolated.append(interp_sim)

            if interpolated:
                mean_sim = np.mean(interpolated, axis=0)
                std_sim = np.std(interpolated, axis=0)

                # 통계 저장
                robust_stats[num_drones] = {
                    'mean_final': mean_sim[-1],
                    'mean_overall': np.mean(mean_sim),
                    'n_tests': len(filtered_data)
                }

                # 평균 곡선
                ax.plot(time_grid, mean_sim,
                       color=colors[num_drones],
                       linewidth=3.0,
                       label=f'{num_drones} ugvs',
                       alpha=0.9,
                       zorder=10-num_drones)

                # 표준편차 영역
                ax.fill_between(time_grid,
                               mean_sim - std_sim,
                               mean_sim + std_sim,
                               color=colors[num_drones],
                               alpha=0.15,
                               zorder=5-num_drones)

                print(f"  ✓ {len(filtered_data)}개 테스트 사용 (이상치 제거 후)")
                print(f"    - 평균 similarity: {np.mean(mean_sim):.4f}")
                print(f"    - 최종 similarity: {mean_sim[-1]:.4f}")

    # 그래프 스타일링
    ax.set_xlabel('Time [s]', fontweight='bold', fontsize=20)
    ax.set_ylabel('Formation Similarity', fontweight='bold', fontsize=20)

    # Grid
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.8)
    ax.set_axisbelow(True)

    # Axis limits - Fixed time range for cleaner comparison
    ax.set_xlim([0, 29])  # All experiments complete within 20s, +10s for stability
    ax.set_ylim([0, 0.25])

    # Legend
    legend = ax.legend(loc='upper right',
                      fontsize=18,
                      framealpha=0.95,
                      edgecolor='black',
                      fancybox=False,
                      shadow=False)
    legend.get_frame().set_linewidth(1.5)

    # Tick parameters
    ax.tick_params(axis='both', labelsize=17, width=1.5, length=6)

    # Spines
    for spine in ax.spines.values():
        spine.set_linewidth(1.5)

    plt.tight_layout()

    # 저장
    output_base = output_dir / 'convergence_overlay_robust'

    plt.savefig(f'{output_base}.png', dpi=600, bbox_inches='tight',
               facecolor='white', edgecolor='none')
    plt.savefig(f'{output_base}.pdf', dpi=600, bbox_inches='tight',
               facecolor='white', edgecolor='none')

    print("\n" + "="*80)
    print("저장 완료:")
    print(f"  - {output_base}.png (600 DPI)")
    print(f"  - {output_base}.pdf (vector)")
    print("\n통계:")
    for n, stats in robust_stats.items():
        print(f"  {n}대: final={stats['mean_final']:.4f}, n={stats['n_tests']}")
    print("="*80)

    plt.close()


def main():
    plot_convergence_overlay_robust()


if __name__ == "__main__":
    main()
