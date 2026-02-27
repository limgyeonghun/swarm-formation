#!/usr/bin/env python3
import re
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime

def moving_average(data, window_size):
    """Apply moving average filter to smooth the data."""
    if len(data) < window_size:
        return data

    # Use numpy convolution for efficient moving average
    kernel = np.ones(window_size) / window_size
    smoothed = np.convolve(data, kernel, mode='valid')

    # Pad the beginning to maintain original length
    padding = np.full(window_size - 1, smoothed[0])
    return np.concatenate([padding, smoothed])

def parse_log_file(log_path):
    """Parse a single log file and extract timestamp, similarity values, formation cost, jerk, and formation command events."""
    timestamps = []
    similarities = []
    formation_costs = []
    total_jerks = []
    max_jerks = []
    formation_changes = []

    # Patterns to extract various metrics
    similarity_pattern = r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\].*similarity=([\d.]+)'
    formation_cost_pattern = r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\].*formation_cost=([\d.]+)'
    jerk_pattern = r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\].*total_jerk=([\d.]+).*max_jerk=([\d.]+)'

    # First pass: find all formation command timestamps (including step changes)
    with open(log_path, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            # Check for formation command
            if 'Received formation command' in line:
                timestamp_match = re.search(r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]', line)

                # Look for the formation type in the next line
                if i + 1 < len(lines):
                    next_line = lines[i + 1]
                    formation_match = re.search(r'Formation command: (\S+), scale', next_line)

                    if timestamp_match and formation_match:
                        timestamp_str = timestamp_match.group(1)
                        formation_type = formation_match.group(1)
                        timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
                        formation_changes.append({
                            'time': timestamp,
                            'type': formation_type
                        })

    # Second pass: extract all metrics (similarity, formation_cost, jerk)
    with open(log_path, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            # Extract similarity and formation_cost (they appear together)
            sim_match = re.search(similarity_pattern, line)
            fcost_match = re.search(formation_cost_pattern, line)

            if sim_match and fcost_match:
                timestamp_str = sim_match.group(1)
                similarity = float(sim_match.group(2))
                formation_cost = float(fcost_match.group(2))

                # Parse timestamp
                timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')

                timestamps.append(timestamp)
                similarities.append(similarity)
                formation_costs.append(formation_cost)

            # Extract jerk metrics (appears on separate line before COST line)
            jerk_match = re.search(jerk_pattern, line)
            if jerk_match:
                total_jerk = float(jerk_match.group(2))
                max_jerk = float(jerk_match.group(3))

                total_jerks.append(total_jerk)
                max_jerks.append(max_jerk)

    return timestamps, similarities, formation_costs, total_jerks, max_jerks, formation_changes

def time_warp_with_anchors(timestamps, data_arrays, anchor_times, target_anchor_times):
    """
    Time-warp the data using anchor points (formation changes).
    Each segment between anchors is scaled to match the target segment duration.
    data_arrays: list of data arrays to warp (e.g., [similarities, formation_costs, ...])
    Returns: warped_times, list of warped data arrays
    """
    if not timestamps or not anchor_times or len(anchor_times) != len(target_anchor_times):
        return [], [[] for _ in data_arrays]

    warped_times = []
    warped_data_arrays = [[] for _ in data_arrays]

    # Add start and end anchors
    all_anchors = [timestamps[0]] + anchor_times + [timestamps[-1]]
    target_anchors = [target_anchor_times[0] - (anchor_times[0] - timestamps[0]).total_seconds()] + \
                     target_anchor_times + \
                     [target_anchor_times[-1] + (timestamps[-1] - anchor_times[-1]).total_seconds()]

    for i in range(len(timestamps)):
        t = timestamps[i]

        # Find which segment this timestamp belongs to
        segment_idx = 0
        for j in range(len(all_anchors) - 1):
            if all_anchors[j] <= t <= all_anchors[j + 1]:
                segment_idx = j
                break

        # Calculate normalized position within the segment (0 to 1)
        segment_start = all_anchors[segment_idx]
        segment_end = all_anchors[segment_idx + 1]
        segment_duration = (segment_end - segment_start).total_seconds()

        if segment_duration > 0:
            position_in_segment = (t - segment_start).total_seconds() / segment_duration
        else:
            position_in_segment = 0

        # Map to target segment
        target_start = target_anchors[segment_idx]
        target_end = target_anchors[segment_idx + 1]
        warped_time = target_start + position_in_segment * (target_end - target_start)

        warped_times.append(warped_time)
        for idx, data_array in enumerate(data_arrays):
            if i < len(data_array):
                warped_data_arrays[idx].append(data_array[i])

    return warped_times, warped_data_arrays

def process_directory(dir_path, reference_anchor_times=None, reference_formation_types=None):
    """
    Process all ugv logs in a directory and return averaged similarity over time.
    If reference_anchor_times is provided, time-warp the data to align with reference formation changes.
    """
    ugv_data = []
    all_formation_changes = []

    for ugv_id in range(1, 5):  # ugv1, ugv2, ugv3, ugv4
        ugv_dir = dir_path / f'ugv{ugv_id}' / 'logs' / 'runtime'

        # Find replan_fsm log file (drone ID = ugv_id - 1)
        drone_id = ugv_id - 1
        log_files = list(ugv_dir.glob(f'replan_fsm_drone_{drone_id}_*.log'))

        if log_files:
            log_file = log_files[0]  # Take the first one if multiple exist
            print(f"Processing: {log_file}")

            timestamps, similarities, formation_changes = parse_log_file(log_file)

            if timestamps and formation_changes:
                all_formation_changes.append(formation_changes)
                ugv_data.append((timestamps, similarities, formation_changes))
        else:
            print(f"Warning: No log file found in {ugv_dir}")

    # Average across all UGVs
    if not ugv_data:
        return [], [], [], []

    # Extract formation change times and types from the first UGV (assuming all UGVs have similar timing)
    anchor_times = [fc['time'] for fc in all_formation_changes[0]]
    formation_types = [fc['type'] for fc in all_formation_changes[0]]

    # If no reference provided, use the first UGV's formation changes as reference
    if reference_anchor_times is None:
        # Convert to relative times from the first formation change
        if anchor_times:
            reference_time = anchor_times[0]
            reference_anchor_times = [0.0] + [(t - reference_time).total_seconds() for t in anchor_times[1:]]
        else:
            reference_anchor_times = []
        reference_formation_types = formation_types

    print(f"  Formation commands: {len(anchor_times)}")
    for i, fc in enumerate(all_formation_changes[0]):
        print(f"    {i+1}. {fc['type']} at {fc['time']}")

    # Time-warp or normalize all UGV data
    warped_data = []
    for timestamps, similarities, formation_changes in ugv_data:
        fc_times = [fc['time'] for fc in formation_changes]

        if reference_anchor_times and fc_times:
            # Time-warp using formation changes as anchors
            warped_times, warped_sims = time_warp_with_anchors(
                timestamps, similarities, fc_times, reference_anchor_times
            )
            warped_data.append((warped_times, warped_sims))
        elif fc_times:
            # Simple normalization relative to first formation change
            ref_time = fc_times[0]
            relative_times = [(t - ref_time).total_seconds() for t in timestamps]
            warped_data.append((relative_times, similarities))

    # Find the minimum length to align all data
    if not warped_data:
        return [], [], reference_anchor_times, reference_formation_types

    min_length = min(len(data[0]) for data in warped_data)

    # Truncate all data to the same length and average
    avg_times = warped_data[0][0][:min_length]
    avg_similarities = np.mean([data[1][:min_length] for data in warped_data], axis=0)

    return avg_times, avg_similarities, reference_anchor_times, reference_formation_types

def main():
    # Set publication-quality style (matching curvature plot style)
    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 15,
        'axes.labelsize': 20,
        'axes.titlesize': 20,
        'xtick.labelsize': 15,
        'ytick.labelsize': 15,
        'legend.fontsize': 20,
        'axes.linewidth': 1.2,
        'grid.linewidth': 0.5,
        'lines.linewidth': 2.5,
        'savefig.dpi': 600,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.05
    })

    base_dir = Path('logs/jetson/real')

    # Define experiments: Simulation vs Real for each obstacle condition
    experiments = {
        'Without Obstacles': {
            'Simulation': 'jetson_simul_1208_no_obs',
            'Real': 'jetson_real_1208_no_obs_1'
        },
        'With Obstacles': {
            'Simulation': 'jetson_simul_1208_obs',
            'Real': 'jetson_real_1208_obs'
        }
    }

    # Professional color scheme (Nature/Science journal style, colorblind-safe)
    sim_color = '#E64B35'  # Vibrant red for simulation
    real_color = '#4DBBD5'  # Cyan-blue for real
    shade_color = '#FFD700'  # Gold/yellow for shading (much more visible)

    # First pass: collect all data to determine common axis ranges
    all_data = {}
    max_time = 0
    max_similarity = 0

    for obstacle_condition, exp_dict in experiments.items():
        sim_dir = base_dir / exp_dict['Simulation']
        sim_times, sim_similarities, ref_anchors, ref_types = process_directory(sim_dir)

        real_dir = base_dir / exp_dict['Real']
        real_times, real_similarities, _, _ = process_directory(real_dir, ref_anchors, ref_types)

        all_data[obstacle_condition] = {
            'sim_times': sim_times,
            'sim_similarities': sim_similarities,
            'real_times': real_times,
            'real_similarities': real_similarities,
            'ref_anchors': ref_anchors,
            'ref_types': ref_types
        }

        # Track maximum values
        if len(sim_times) > 0:
            max_time = max(max_time, max(sim_times))
        if len(real_times) > 0:
            max_time = max(max_time, max(real_times))
        if len(sim_similarities) > 0:
            max_similarity = max(max_similarity, max(sim_similarities))
        if len(real_similarities) > 0:
            max_similarity = max(max_similarity, max(real_similarities))

    # Set common axis limits with padding
    common_xlim = (-max_time * 0.02, max_time * 1.02)
    common_ylim = (0, max_similarity * 1.1)

    # Second pass: create plots with common axis ranges
    for obstacle_condition in experiments.keys():
        # Create a separate figure for each condition
        fig, ax = plt.subplots(figsize=(14, 8))

        print(f"\n{'='*60}")
        print(f"Processing: {obstacle_condition}")
        print(f"{'='*60}")

        # Get pre-processed data
        data = all_data[obstacle_condition]
        sim_times = data['sim_times']
        sim_similarities = data['sim_similarities']
        real_times = data['real_times']
        real_similarities = data['real_similarities']
        reference_anchors = data['ref_anchors']
        reference_formation_types = data['ref_types']

        # Add shaded regions for formation transitions BEFORE plotting lines
        if reference_anchors and reference_formation_types:
            # Add shaded regions for step1->step2 transitions (formation stabilizing)
            transition_label_added = False
            for i in range(len(reference_anchors) - 1):
                current_formation = reference_formation_types[i]
                next_formation = reference_formation_types[i + 1]

                # Shade only when formation type is SAME (step1 -> step2 within same formation)
                # This is the period where formation is stabilizing after initial change
                is_step1_to_step2 = (current_formation == next_formation)

                if is_step1_to_step2:
                    # Shade the transition region (step1 -> step2)
                    start_time = reference_anchors[i]
                    end_time = reference_anchors[i + 1]
                    label = 'Formation transition' if not transition_label_added else ''
                    ax.axvspan(start_time, end_time, alpha=0.35, color=shade_color,
                              edgecolor='orange', linewidth=2.0, zorder=0, label=label)
                    if label:
                        transition_label_added = True

        # Apply moving average filter
        window_size = 5  # Adjust this value to control smoothness (higher = smoother)

        if len(sim_times) > 0:
            # Apply moving average to simulation data
            sim_smoothed = moving_average(np.array(sim_similarities), window_size)
            ax.plot(sim_times, sim_smoothed, color=sim_color,
                   linewidth=2.5, alpha=0.9, label='Simulation', zorder=3)

            # Print statistics
            mean_sim = np.mean(sim_smoothed)
            max_sim = np.max(sim_smoothed)
            final_sim = sim_smoothed[-1] if len(sim_smoothed) > 0 else 0

            print(f"  Simulation statistics:")
            print(f"    Mean similarity: {mean_sim:.6f}")
            print(f"    Max similarity: {max_sim:.6f}")
            print(f"    Final similarity: {final_sim:.6f}")
            print(f"    Duration: {sim_times[-1]:.2f} seconds")
            print(f"    Data points: {len(sim_times)}")

        if len(real_times) > 0:
            # Apply moving average to real data
            real_smoothed = moving_average(np.array(real_similarities), window_size)
            ax.plot(real_times, real_smoothed, color=real_color,
                   linewidth=2.5, alpha=0.9, label='Real world', zorder=3)

            # Print statistics
            mean_sim = np.mean(real_smoothed)
            max_sim = np.max(real_smoothed)
            final_sim = real_smoothed[-1] if len(real_smoothed) > 0 else 0

            print(f"  Real statistics:")
            print(f"    Mean similarity: {mean_sim:.6f}")
            print(f"    Max similarity: {max_sim:.6f}")
            print(f"    Final similarity: {final_sim:.6f}")
            print(f"    Duration: {real_times[-1]:.2f} seconds")
            print(f"    Data points: {len(real_times)}")

        # Add vertical lines for formation changes
        if reference_anchors:
            for i, anchor_time in enumerate(reference_anchors):
                ax.axvline(x=anchor_time, color='#888888', linestyle='--',
                          alpha=0.4, linewidth=1.0, zorder=1)

        # Configure plot with professional style (matching curvature plot)
        ax.set_xlabel('Time [s]', fontweight='bold', fontsize=20)
        ax.set_ylabel('Formation Similarity', fontweight='bold', fontsize=20)
        ax.set_title(f'{obstacle_condition}', fontweight='bold', fontsize=20, pad=10)

        # Professional grid
        ax.grid(True, alpha=0.3)
        ax.set_axisbelow(True)

        # Set common axis limits for both plots
        ax.set_xlim(common_xlim)
        ax.set_ylim(common_ylim)

        # Legend with better positioning (matching curvature plot style)
        ax.legend(loc='upper left', bbox_to_anchor=(0.01, 0.99), fontsize=20,
                 framealpha=0.95, edgecolor='black', fancybox=False, shadow=False)

        # Tick parameters
        ax.tick_params(axis='both', labelsize=15)

        # Adjust layout
        plt.tight_layout()

        # Save in multiple formats for publication
        # Use a different filename for each condition
        condition_suffix = 'no_obs' if 'Without' in obstacle_condition else 'with_obs'
        output_base = f'jetson_similarity_{condition_suffix}'

        # High-resolution PNG for general use
        plt.savefig(f'{output_base}.png', dpi=600, bbox_inches='tight', facecolor='white')

        # PDF for LaTeX/publication (vector format)
        plt.savefig(f'{output_base}.pdf', dpi=600, bbox_inches='tight', facecolor='white')

        # EPS for some journals (vector format)
        plt.savefig(f'{output_base}.eps', dpi=600, bbox_inches='tight', facecolor='white')

        print(f"\nFigures saved:")
        print(f"  - {output_base}.png (raster, 600 DPI)")
        print(f"  - {output_base}.pdf (vector, for LaTeX)")
        print(f"  - {output_base}.eps (vector, for journals)")

        # Show the plot (optional - comment out if running headless)
        # plt.show()

        # Close the figure to free memory
        plt.close(fig)

    print(f"\n{'='*60}")
    print(f"All figures saved successfully!")
    print(f"{'='*60}")

if __name__ == '__main__':
    main()
