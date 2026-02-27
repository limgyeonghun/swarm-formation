#!/usr/bin/env python3
"""
Runtime logs에서 replan별 곡률 추출 및 시각화

매 replan마다 최대 곡률과 평균 곡률을 추출하여 시간에 따른 변화 그래프 생성
"""

import re
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Headless mode
import matplotlib.pyplot as plt
from datetime import datetime
from typing import List, Dict, Tuple

def parse_timestamp(log_line: str) -> float:
    """로그 라인에서 타임스탬프 추출 (초 단위)"""
    # 로그 형식: [2025-12-11 02:34:03.330] [INFO ] [replan_fsm_drone_0] ...
    match = re.search(r'\[(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}\.\d+)\]', log_line)
    if match:
        timestamp_str = match.group(1)
        # Convert to datetime then to float (seconds since epoch)
        dt = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
        return dt.timestamp()
    return None


def extract_curvature_per_replan(log_file: str) -> List[Dict]:
    """
    Runtime log에서 replan별 곡률 데이터 추출

    각 replan마다 NONHOLONOMIC SUMMARY (마지막 결과)만 추출하여
    중복 카운트를 방지합니다.

    Returns:
        List of dicts with keys: replan_id, timestamp, max_curvature, mean_curvature, curvature_violations
    """
    curvature_data = []

    with open(log_file, 'r') as f:
        lines = f.readlines()

    current_replan_id = None
    in_replan = False

    # Temporary storage for current replan's data
    temp_max_curv = None
    temp_mean_curv = None
    temp_curv_violations = None
    temp_timestamp = None

    for i, line in enumerate(lines):
        # Detect replan start
        replan_match = re.search(r'=== DRONE \d+ REPLAN (\d+) START ===', line)
        if replan_match:
            current_replan_id = int(replan_match.group(1))
            in_replan = True
            temp_timestamp = parse_timestamp(line)
            # Reset temp data
            temp_max_curv = None
            temp_mean_curv = None
            temp_curv_violations = None
            continue

        # Detect replan completion (this marks the end, we should have all data by now)
        if in_replan and '=== DRONE' in line and 'REPLAN COMPLETED' in line:
            # Save the data if we have curvature info
            if temp_max_curv is not None and current_replan_id is not None:
                curvature_data.append({
                    'replan_id': current_replan_id,
                    'timestamp': temp_timestamp,
                    'max_curvature': temp_max_curv,
                    'mean_curvature': temp_mean_curv,
                    'curvature_violations': temp_curv_violations  # Only curvature violations
                })

            in_replan = False
            current_replan_id = None
            continue

        # Extract curvature data from NONHOLONOMIC VIOLATIONS sections
        # We update temp variables each time, so the last one (final iteration) will be kept
        if in_replan and '[NONHOLONOMIC VIOLATIONS]' in line:
            # Look for curvature line in next few lines
            for j in range(i+1, min(i+10, len(lines))):
                curv_line = lines[j]

                # curvature: violations=X, cost=X.X, max_κ=X.X, mean_κ=X.X (limit=X.X)
                if 'curvature:' in curv_line:
                    viol_match = re.search(r'violations=(\d+)', curv_line)
                    max_match = re.search(r'max_κ=([\d.]+)', curv_line)
                    mean_match = re.search(r'mean_κ=([\d.]+)', curv_line)

                    if viol_match:
                        temp_curv_violations = int(viol_match.group(1))
                    if max_match:
                        temp_max_curv = float(max_match.group(1))
                    if mean_match:
                        temp_mean_curv = float(mean_match.group(1))
                    break

    return curvature_data


def plot_curvature_comparison(data_w0: List[Dict], data_w1500: List[Dict], output_file: str = 'curvature_timeline.png'):
    """
    곡률 비교 그래프 생성 (inset으로 확대 영역 표시)

    Args:
        data_w0: Weight=0 데이터 (제약 미고려)
        data_w1500: Weight=1500 데이터 (통합 최적화)
        output_file: 출력 파일명
    """
    # Set font to support Korean
    plt.rcParams['font.family'] = 'DejaVu Sans'

    # Use replan index instead of time
    if data_w0:
        max_curv_w0 = [d['max_curvature'] for d in data_w0]
    else:
        max_curv_w0 = []

    if data_w1500:
        max_curv_w1500 = [d['max_curvature'] for d in data_w1500]
    else:
        max_curv_w1500 = []

    # Find common length (trim to shorter length)
    max_replans = min(len(max_curv_w0), len(max_curv_w1500))

    # Trim data to common length
    if max_curv_w0:
        max_curv_w0 = max_curv_w0[:max_replans]

    if max_curv_w1500:
        max_curv_w1500 = max_curv_w1500[:max_replans]

    # Create replan indices
    replan_indices = list(range(max_replans))

    # Create main plot
    fig, ax_main = plt.subplots(figsize=(14, 8))

    # Plot main graph (full range)
    if replan_indices:
        ax_main.plot(replan_indices, max_curv_w0, 'o-', color='red', alpha=0.5,
                    label='Without constraint', linewidth=1.5, markersize=3)
        ax_main.plot(replan_indices, max_curv_w1500, 's-', color='blue', alpha=0.5,
                    label='With constraint (Ours)', linewidth=1.5, markersize=3)

    # Add horizontal limit lines
    ax_main.axhline(y=0.4, color='black', linestyle='--', linewidth=2.0,
                   label=r'Design target ($\kappa_{design}=0.4$)', zorder=10)
    ax_main.axhline(y=2.0, color='grey', linestyle='-.', linewidth=2.5,
                   label=r'Physical limit ($\kappa_{physical}=2.0$)', zorder=10)

    # Set labels (no title)
    ax_main.set_xlabel('Replan cycle', fontsize=20, fontweight='bold')
    ax_main.set_ylabel('Curvature [1/m]', fontsize=20, fontweight='bold')

    # Set limits
    ax_main.set_ylim(-5, 165)
    if max_replans > 0:
        ax_main.set_xlim(-10, max_replans + 10)

    # Add legend
    ax_main.legend(loc='upper left', bbox_to_anchor=(0.01, 0.99), fontsize=16, framealpha=0.95, edgecolor='black')

    # Grid
    ax_main.grid(False, alpha=0.0)
    ax_main.tick_params(axis='both', labelsize=15)

    # Create inset axes for zoomed view
    # Position: [left, bottom, width, height] in figure coordinates (0 to 1)
    ax_inset = fig.add_axes([0.17, 0.32, 0.35, 0.35])

    # Plot zoomed data in inset
    if replan_indices:
        ax_inset.plot(replan_indices, max_curv_w0, 'o-', color='red', alpha=0.5,
                     linewidth=1.5, markersize=2)
        ax_inset.plot(replan_indices, max_curv_w1500, 's-', color='blue', alpha=0.5,
                     linewidth=1.5, markersize=2)

    # Add limit lines to inset
    ax_inset.axhline(y=0.4, color='black', linestyle='--', linewidth=1.5, zorder=10)
    ax_inset.axhline(y=2.0, color='grey', linestyle='-.', linewidth=1.5, zorder=10)

    # Set inset limits
    ax_inset.set_ylim(-0.2, 10)
    if max_replans > 0:
        ax_inset.set_xlim(-10, max_replans + 10)

    # Inset formatting
    # ax_inset.set_title('Zoomed view (0-10)', fontsize=13, fontweight='bold')
    ax_inset.grid(False, alpha=0.0)
    ax_inset.tick_params(axis='both', labelsize=11)

    # Add a box around the inset for clarity
    ax_inset.spines['top'].set_linewidth(2)
    ax_inset.spines['bottom'].set_linewidth(2)
    ax_inset.spines['left'].set_linewidth(2)
    ax_inset.spines['right'].set_linewidth(2)

    plt.savefig(output_file, dpi=600, bbox_inches='tight')
    print(f"✓ Saved plot to {output_file}")
    plt.close()


def plot_replan_bar_chart(data_w0: List[Dict], data_w1500: List[Dict], output_file: str = 'curvature_replan_bars.png'):
    """
    Replan별 최대 곡률 막대 그래프 (처음 20개만)

    Args:
        data_w0: Weight=0 데이터
        data_w1500: Weight=1500 데이터
        output_file: 출력 파일명
    """
    # Limit to first 20 replans for readability
    num_replans = min(20, len(data_w0), len(data_w1500))

    if num_replans == 0:
        print("Not enough data for bar chart")
        return

    replan_ids = np.arange(num_replans)
    max_curv_w0 = [data_w0[i]['max_curvature'] for i in range(num_replans)]
    max_curv_w1500 = [data_w1500[i]['max_curvature'] for i in range(num_replans)]

    fig, ax = plt.subplots(figsize=(14, 6))

    width = 0.35
    x = np.arange(num_replans)

    bars1 = ax.bar(x - width/2, max_curv_w0, width, label='No Constraint (w=0)', color='red', alpha=0.7)
    bars2 = ax.bar(x + width/2, max_curv_w1500, width, label='Integrated (w=1500)', color='blue', alpha=0.7)

    ax.axhline(y=0.4, color='green', linestyle='--', linewidth=2, label='Curvature Limit (κ=0.4)')

    ax.set_xlabel('Replan Index', fontsize=24)
    ax.set_ylabel('Max Curvature [1/m]', fontsize=24)
    ax.set_title('Maximum Curvature per Replan (First 20)', fontsize=24, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels([f'{i}' for i in range(num_replans)])
    ax.legend(fontsize=20)
    ax.grid(False, alpha=0.0, axis='y')
    ax.tick_params(axis='x', labelsize=24)
    ax.tick_params(axis='y', labelsize=24)

    plt.tight_layout()
    plt.savefig(output_file, dpi=600, bbox_inches='tight')
    print(f"✓ Saved bar chart to {output_file}")
    plt.close()


def print_statistics(data: List[Dict], label: str):
    """데이터 통계 출력 (곡률 위반만 체크)"""
    if not data:
        print(f"\n{label}: No data")
        return

    max_curvatures = [d['max_curvature'] for d in data]
    mean_curvatures = [d['mean_curvature'] for d in data if d['mean_curvature'] is not None]

    # Count replans with curvature violations (violations > 0)
    curvature_violation_count = sum(1 for d in data if d.get('curvature_violations', 0) > 0)

    # Count exceedances beyond acceptable bound (κ > 2.0)
    # 0.4 is the design limit, 2.0 is the acceptable upper bound
    # Beyond 2.0 represents physically untrackable trajectories
    acceptable_bound = 2.0
    beyond_acceptable_count = sum(1 for m in max_curvatures if m > acceptable_bound)

    print(f"\n{label}:")
    print(f"  Number of replans: {len(data)}")
    print(f"  Max curvature - Overall max: {max(max_curvatures):.1f}, Mean: {np.mean(max_curvatures):.2f}, Std: {np.std(max_curvatures):.2f}")
    if mean_curvatures:
        print(f"  Mean curvature - Overall mean: {np.mean(mean_curvatures):.2f}, Std: {np.std(mean_curvatures):.2f}")
    print(f"  Curvature violations (reported by optimizer): {curvature_violation_count}/{len(data)} replans ({curvature_violation_count/len(data)*100:.1f}%)")
    print(f"  Beyond acceptable bound (κ>{acceptable_bound}): {beyond_acceptable_count}/{len(data)} replans ({beyond_acceptable_count/len(data)*100:.1f}%)")


def main():
    """메인 실행 함수"""
    base_dir = "/home/lim/workspace/ros_ws/swarm-formation/logs/runtime/compare_4rover"

    # Log files
    log_w0 = os.path.join(base_dir, "weight_0/replan_fsm_drone_3_20251211_023340.log")
    log_w1500 = os.path.join(base_dir, "weight_1000/replan_fsm_drone_3_20251211_024258.log")

    # Check files exist
    if not os.path.exists(log_w0):
        print(f"✗ File not found: {log_w0}")
        return
    if not os.path.exists(log_w1500):
        print(f"✗ File not found: {log_w1500}")
        return

    print("Extracting curvature data from runtime logs...")
    print(f"✓ Weight=0 log: {os.path.basename(log_w0)}")
    print(f"✓ Weight=1500 log: {os.path.basename(log_w1500)}")

    # Extract data
    data_w0 = extract_curvature_per_replan(log_w0)
    data_w1500 = extract_curvature_per_replan(log_w1500)

    print(f"\n✓ Extracted {len(data_w0)} replans from Weight=0")
    print(f"✓ Extracted {len(data_w1500)} replans from Weight=1500")

    # Print statistics
    print_statistics(data_w0, "Weight=0 (No Constraint)")
    print_statistics(data_w1500, "Weight=1500 (Integrated)")

    # Generate plots
    print("\nGenerating plots...")
    plot_curvature_comparison(data_w0, data_w1500, 'curvature_timeline.png')
    plot_replan_bar_chart(data_w0, data_w1500, 'curvature_replan_bars.png')

    print("\n✓ Analysis complete!")


if __name__ == "__main__":
    main()
