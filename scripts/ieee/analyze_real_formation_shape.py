#!/usr/bin/env python3
"""
Analyze real flight formation shape similarity.
(Modified to use Jetson FSM logs for positions instead of PX4 ULogs)
"""

import re
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime
from scipy.spatial.distance import pdist, squareform
from scipy.optimize import linear_sum_assignment
from scipy.interpolate import interp1d
import argparse
import glob


class ShapeAnalyzer:
    """Shape-based formation similarity analyzer (scale/rotation/translation invariant)"""

    @staticmethod
    def normalize_shape(positions):
        """
        Normalize shape to be invariant to translation, rotation, and scale.
        """
        # Center the shape (translation invariant)
        centered = positions - np.mean(positions, axis=0)

        # Get pairwise distances
        distances = pdist(centered, metric='euclidean')
        dist_matrix = squareform(distances)

        # Normalize by characteristic length (scale invariant)
        char_length = np.mean(distances[distances > 0])
        if char_length < 1e-6:
            return dist_matrix

        normalized_dist = dist_matrix / char_length

        return normalized_dist

    @staticmethod
    def shape_similarity(actual_pos, target_pos):
        """
        Calculate shape similarity between actual and target formations.
        Returns value in [0, 1] where 0 = perfect match.
        """
        if len(actual_pos) != len(target_pos):
            return 1.0

        # Normalize both shapes
        actual_norm = ShapeAnalyzer.normalize_shape(actual_pos)
        target_norm = ShapeAnalyzer.normalize_shape(target_pos)

        # Find best permutation (Hungarian algorithm)
        n = len(actual_pos)
        cost_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(n):
                cost_matrix[i, j] = np.sum(np.abs(actual_norm[i, :] - target_norm[j, :]))

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        permuted_actual = actual_norm[row_ind, :][:, row_ind]
        permuted_target = target_norm[col_ind, :][:, col_ind]

        diff = permuted_actual - permuted_target
        similarity = np.sqrt(np.mean(diff**2))

        return similarity

    @staticmethod
    def side_length_error(actual_pos, target_pos):
        if len(actual_pos) != len(target_pos):
            return 1.0, 1.0

        actual_dist = pdist(actual_pos)
        target_dist = pdist(target_pos)

        with np.errstate(divide='ignore', invalid='ignore'):
            rel_error = np.abs(actual_dist - target_dist) / (target_dist + 1e-6)

        return np.mean(rel_error), np.max(rel_error)


def extract_fsm_positions(jetson_log_dir, resample_dt=0.1):
    """
    Extract drone positions from Jetson FSM logs.
    Parses lines like: [TIMESTAMP] ... [replan_fsm_drone_X] Start: (x,y,z) ...
    """
    drone_data = {}
    
    # Find all replan_fsm logs recursively
    log_files = glob.glob(str(Path(jetson_log_dir) / "**" / "replan_fsm_drone_*.log"), recursive=True)
    
    if not log_files:
        print(f"No FSM logs found in {jetson_log_dir}")
        return {}

    print(f"Found {len(log_files)} FSM logs. Parsing positions...")
    
    # Regex to capture timestamp, drone_id, and start position (x, y, z)
    # Example: [2025-12-08 08:31:51.596] [INFO ] [replan_fsm_drone_3] Start: (32.51,-92.62,-0.00)
    pos_pattern = re.compile(r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\].*\[replan_fsm_drone_(\d+)\].*Start: \(([-\d.]+),([-\d.]+),([-\d.]+)\)')

    for log_file in log_files:
        with open(log_file, 'r') as f:
            for line in f:
                if "Start:" in line and "replan_fsm_drone" in line:
                    match = pos_pattern.search(line)
                    if match:
                        ts_str, drone_id_str, x_str, y_str, z_str = match.groups()
                        
                        # Parse timestamp
                        dt = datetime.strptime(ts_str, '%Y-%m-%d %H:%M:%S.%f')
                        ts = dt.timestamp()
                        
                        # Parse drone ID
                        d_id = int(drone_id_str)
                        
                        if d_id not in drone_data:
                            drone_data[d_id] = {'t': [], 'x': [], 'y': [], 'z': []}
                        
                        drone_data[d_id]['t'].append(ts)
                        drone_data[d_id]['x'].append(float(x_str))
                        drone_data[d_id]['y'].append(float(y_str))
                        drone_data[d_id]['z'].append(float(z_str))

    # Normalize Drone IDs to 0-based index (0, 1, 2, 3...)
    if not drone_data:
        return {}

    sorted_ids = sorted(drone_data.keys())
    id_map = {uid: i for i, uid in enumerate(sorted_ids)}
    print(f"  -> Mapped Drone IDs: {id_map}")

    # Determine common time range
    all_ts = []
    for d_id in drone_data:
        all_ts.extend(drone_data[d_id]['t'])
    
    if not all_ts:
        return {}

    t_min = min(all_ts)
    t_max = max(all_ts)
    
    print(f"\nResampling FSM data to {1/resample_dt:.0f}Hz (dt={resample_dt}s)")
    print(f"  Time range: {datetime.fromtimestamp(t_min)} to {datetime.fromtimestamp(t_max)}")
    
    resampled_timestamps = np.arange(t_min, t_max, resample_dt)
    positions_dict = {}

    for d_id, data in drone_data.items():
        mapped_id = id_map[d_id]
        
        # Sort data by time
        sorted_indices = np.argsort(data['t'])
        t_arr = np.array(data['t'])[sorted_indices]
        x_arr = np.array(data['x'])[sorted_indices]
        y_arr = np.array(data['y'])[sorted_indices]
        z_arr = np.array(data['z'])[sorted_indices]
        
        # Remove duplicates
        _, unique_indices = np.unique(t_arr, return_index=True)
        t_arr = t_arr[unique_indices]
        x_arr = x_arr[unique_indices]
        y_arr = y_arr[unique_indices]
        z_arr = z_arr[unique_indices]

        if len(t_arr) < 2:
            print(f"  Warning: Not enough data points for drone {d_id}")
            continue

        # Interpolate
        fx = interp1d(t_arr, x_arr, kind='linear', fill_value="extrapolate")
        fy = interp1d(t_arr, y_arr, kind='linear', fill_value="extrapolate")
        fz = interp1d(t_arr, z_arr, kind='linear', fill_value="extrapolate")

        ix = fx(resampled_timestamps)
        iy = fy(resampled_timestamps)
        iz = fz(resampled_timestamps)

        for i, t in enumerate(resampled_timestamps):
            if t not in positions_dict:
                positions_dict[t] = {}
            positions_dict[t][mapped_id] = (ix[i], iy[i], iz[i])

    print(f"  Resampled to {len(resampled_timestamps)} synchronized timestamps for {len(drone_data)} drones")

    return positions_dict


def extract_formation_commands(log_file):
    """
    Extract formation patterns from Jetson log file. Handles 'line' 
    formation as a special case which might not have [POLY_TRAJ_OPT].
    """
    formations = []
 
    with open(log_file, 'r') as f:
        lines = f.readlines()
 
    i = 0
    while i < len(lines):
        line = lines[i]
        # Find the line that indicates a formation change command
        type_match = re.search(r'Formation command: (\w+)', line)
        if type_match:
            current_formation_type = type_match.group(1)
            
            cmd_timestamp_match = re.search(r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]', line)
            if not cmd_timestamp_match:
                i += 1
                continue
            cmd_timestamp = datetime.strptime(cmd_timestamp_match.group(1), '%Y-%m-%d %H:%M:%S.%f').timestamp()

            positions_found = False
            # Search forward for the corresponding position data
            for k in range(i + 1, min(i + 50, len(lines))):
                if 'Formation Positions:' in lines[k] and '[POLY_TRAJ_OPT]' in lines[k]:
                    timestamp_match = re.search(r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]', lines[k])
                    if timestamp_match:
                        pos_timestamp = datetime.strptime(timestamp_match.group(1), '%Y-%m-%d %H:%M:%S.%f').timestamp()
                        
                        # If the position log is too far from the command, it's likely for a different command
                        if (pos_timestamp - cmd_timestamp) > 5.0:
                            break

                        pattern = []
                        j = k + 1
                        while j < len(lines) and j < k + 15:
                            pos_match = re.search(r'Drone \d+:\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]', lines[j])
                            if pos_match:
                                pattern.append((float(pos_match.group(1)),
                                                float(pos_match.group(2)),
                                                float(pos_match.group(3))))
                            elif '=====' in lines[j]:
                                break
                            j += 1
                        
                        if pattern:
                            formations.append((pos_timestamp, current_formation_type, pattern))
                            i = j # Skip ahead to avoid re-processing these lines
                            positions_found = True
                            break # Stop searching for position data for this command
            
            if not positions_found and 'line' in current_formation_type.lower():
                print(f"  -> Found '{current_formation_type}' command at {datetime.fromtimestamp(cmd_timestamp)} without [POLY_TRAJ_OPT]. Using initial offsets as target.")
                formations.append((cmd_timestamp, current_formation_type, 'USE_LINE_DEFAULT'))

        i += 1
 
    # Sort by timestamp to ensure chronological order
    formations.sort(key=lambda x: x[0])
    print(f"Total: {len(formations)} formation updates found in log.")
    return formations


def moving_average(data, window_size):
    """Apply moving average filter to smooth the data."""
    if len(data) < window_size:
        return np.array(data)

    kernel = np.ones(window_size) / window_size
    smoothed = np.convolve(data, kernel, mode='valid')

    # Pad to keep same length
    padding = np.full(window_size - 1, smoothed[0])
    return np.concatenate([padding, smoothed])


def time_warp_with_anchors(time_values, similarity_values, formation_changes, reference_anchor_times):
    """
    Time-warp the data using anchor points (formation changes).
    Each segment between anchors is scaled to match the target segment duration.

    Args:
        time_values: list of time values
        similarity_values: list of similarity values
        formation_changes: list of dicts with 'time' and 'type'
        reference_anchor_times: list of reference anchor times to align to

    Returns:
        warped_times: list of warped time values
        warped_similarities: list of warped similarity values
    """
    if not time_values or not formation_changes:
        return time_values, similarity_values

    if len(formation_changes) != len(reference_anchor_times):
        print(f"  Warning: formation_changes ({len(formation_changes)}) != reference_anchor_times ({len(reference_anchor_times)})")
        return time_values, similarity_values

    anchor_times = [fc['time'] for fc in formation_changes]

    # Add start and end anchors
    all_anchors = [time_values[0]] + anchor_times + [time_values[-1]]

    # Calculate target anchors with same padding
    start_padding = anchor_times[0] - time_values[0]
    end_padding = time_values[-1] - anchor_times[-1]

    target_anchors = [reference_anchor_times[0] - start_padding] + \
                     reference_anchor_times + \
                     [reference_anchor_times[-1] + end_padding]

    warped_times = []
    warped_similarities = []

    for i, t in enumerate(time_values):
        # Find which segment this timestamp belongs to
        segment_idx = 0
        for j in range(len(all_anchors) - 1):
            if all_anchors[j] <= t <= all_anchors[j + 1]:
                segment_idx = j
                break

        # Calculate normalized position within the segment (0 to 1)
        segment_start = all_anchors[segment_idx]
        segment_end = all_anchors[segment_idx + 1]
        segment_duration = segment_end - segment_start

        if segment_duration > 0:
            position_in_segment = (t - segment_start) / segment_duration
        else:
            position_in_segment = 0

        # Map to target segment
        target_start = target_anchors[segment_idx]
        target_end = target_anchors[segment_idx + 1]
        warped_time = target_start + position_in_segment * (target_end - target_start)

        warped_times.append(warped_time)
        if i < len(similarity_values):
            warped_similarities.append(similarity_values[i])

    return warped_times, warped_similarities


def process_single_experiment(jetson_log_dir, resample_dt=0.1):
    """
    Process a single experiment directory and return time-series data.
    Returns: time_values, similarity_values, formation_changes (list of dicts with 'time' and 'type')
    """
    # Assuming Jetson logs follow a pattern like replan_fsm_*.log
    jetson_logs = list(Path(jetson_log_dir).glob('**/replan_fsm_*.log'))
    if not jetson_logs:
        # Fallback search if in simple structure
        jetson_logs = list(Path(jetson_log_dir).glob('replan_fsm_*.log'))

    if not jetson_logs:
        print(f"No Jetson logs found in {jetson_log_dir}")
        return [], [], []

    jetson_log = jetson_logs[0]
    print(f"\nUsing Jetson log: {jetson_log}")

    print("\n" + "="*70)
    print("Extracting formation commands...")
    print("="*70)
    formations_raw = extract_formation_commands(jetson_log)

    # Default line pattern (relative) if specific offsets aren't available
    # Assuming 2m spacing for 4 drones
    line_pattern_2d = np.array([
        [0.0, 0.0], [0.0, 2.0], [0.0, 4.0], [0.0, 6.0]
    ])

    # Process raw formations: replace placeholder and convert to numpy arrays
    formations = []
    for ts, f_type, pattern in formations_raw:
        if isinstance(pattern, str) and pattern == 'USE_LINE_DEFAULT':
            formations.append((ts, f_type, line_pattern_2d))
        else:
            # For other formations, pattern is a list of tuples, convert to numpy array
            formations.append((ts, f_type, np.array(pattern)))

    # Get mission start time from the first formation command
    mission_start_time_abs = formations[0][0] if formations else None
    print(f"Mission start time (from first Jetson command): {datetime.fromtimestamp(mission_start_time_abs).strftime('%Y-%m-%d %H:%M:%S.%f') if mission_start_time_abs else 'Unknown'}")

    # Extract Data from FSM logs
    print("\n" + "="*70)
    print("Extracting position data from FSM logs...")
    print("="*70)
    positions_dict = extract_fsm_positions(jetson_log_dir, resample_dt)

    if not positions_dict or not formations:
        print("Data extraction failed. No positions or formation commands found.")
        return [], [], []

    print("\n" + "="*70)
    print("Calculating shape similarity...")
    print("="*70)

    # --- Time Synchronization using Absolute UNIX Timestamps ---
    timestamps = sorted(positions_dict.keys())
    if not timestamps:
        print("No position data to analyze.")
        return [], [], []

    # Calculate relative times for plotting vertical lines
    formation_times_rel = [(f[0] - mission_start_time_abs) for f in formations]

    # --- Analysis Loop ---
    similarity_values = []
    formation_labels = []
    time_values = []

    num_drones = len(next(iter(positions_dict.values()))) if positions_dict else 0
    current_formation = None
    current_pattern = None
    current_formation_idx = -1

    for t in timestamps:
        positions = positions_dict[t]

        if len(positions) != num_drones:
            continue

        # 't' is the absolute UNIX timestamp from the ULog.
        # Calculate mission time relative to the first command.
        mission_time = t - mission_start_time_abs

        if mission_time < -5: # Ignore data more than 5s before first command
            continue

        new_formation_idx = current_formation_idx
        for i in range(len(formation_times_rel) - 1, -1, -1):
            if mission_time >= formation_times_rel[i]:
                new_formation_idx = i
                break

        if new_formation_idx != current_formation_idx and new_formation_idx >= 0:
            current_formation_idx = new_formation_idx
            current_formation = formations[new_formation_idx][1]
            raw_pattern = formations[new_formation_idx][2]

            # Ensure pattern is 2D for analysis
            if raw_pattern.ndim == 2 and raw_pattern.shape[1] == 3:
                current_pattern = raw_pattern[:, :2]
            else:
                current_pattern = raw_pattern

            print(f"  [{mission_time:6.1f}s] Mission Time: Formation changed to: {current_formation}")

        if current_pattern is None:
            continue

        # Get actual positions (x, y only)
        # These now include the global offsets applied during extraction
        actual_pos = np.array([positions[i][:2] for i in range(num_drones)])

        # Center both shapes for comparison
        actual_pos_centered = actual_pos - np.mean(actual_pos, axis=0)

        similarity = ShapeAnalyzer.shape_similarity(actual_pos_centered, current_pattern)

        similarity_values.append(similarity)
        formation_labels.append(current_formation)
        time_values.append(mission_time)

    # Apply moving average smoothing
    window_size = 5  # 1.0s window (since dt=0.1s)
    if len(similarity_values) > window_size:
        print(f"Applying moving average filter (window size: {window_size})")
        similarity_values = moving_average(similarity_values, window_size)

    # Build formation_changes list (for compatibility with plotting code)
    formation_changes = []
    for i, (ts_abs, f_type, _) in enumerate(formations):
        formation_changes.append({
            'time': formation_times_rel[i],  # relative time
            'type': f_type
        })

    print(f"  Total data points: {len(time_values)}")
    return time_values, similarity_values, formation_changes


def analyze_formation_quality(px4_dir, jetson_log_dir, output_dir='./'):
    """
    Main analysis function - kept for backward compatibility.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True, parents=True)

    time_values, similarity_values, formation_changes = process_single_experiment(jetson_log_dir)

    if not time_values:
        print("No data to plot.")
        return

    # Plot results
    print("\n" + "="*70)
    print("Generating plots...")
    print("="*70)

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(time_values, similarity_values, 'b-', linewidth=1.5, label='Shape Similarity')
    ax.set_ylabel('Formation Similarity', fontsize=12)
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_title('Real Flight Formation Analysis (Using FSM Logs)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()

    for fc in formation_changes:
        ax.axvline(fc['time'], color='red', linestyle='--', alpha=0.5)
        ax.text(fc['time'], ax.get_ylim()[1] * 0.9, fc['type'], rotation=90, va='top', fontsize=9, color='red')

    plt.tight_layout()
    output_file = output_dir / 'real_formation_shape_similarity_fsm.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\nSaved plot: {output_file}")


def main():
    """
    Main function to compare Simulation vs Real formation similarity.
    Uses plot_jetson_similarity.py style with analyze_real_formation_shape.py calculation method.
    """
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

    # First pass: collect all data and apply time warping
    all_data = {}
    max_time = 0
    max_similarity = 0

    for obstacle_condition, exp_dict in experiments.items():
        print(f"\n{'='*60}")
        print(f"Processing: {obstacle_condition}")
        print(f"{'='*60}")

        sim_dir = base_dir / exp_dict['Simulation']
        print(f"\n--- Simulation: {exp_dict['Simulation']} ---")
        sim_times, sim_similarities, sim_formation_changes = process_single_experiment(sim_dir)

        real_dir = base_dir / exp_dict['Real']
        print(f"\n--- Real: {exp_dict['Real']} ---")
        real_times, real_similarities, real_formation_changes = process_single_experiment(real_dir)

        # Use simulation formation changes as reference for time warping
        if sim_formation_changes and real_formation_changes:
            reference_anchor_times = [fc['time'] for fc in sim_formation_changes]
            print(f"\n  Applying time warp to align Real data to Simulation anchors...")
            print(f"    Reference anchors (Simulation): {reference_anchor_times}")
            print(f"    Real anchors before warp: {[fc['time'] for fc in real_formation_changes]}")

            # Time-warp real data to align with simulation anchors
            real_times, real_similarities = time_warp_with_anchors(
                real_times, real_similarities, real_formation_changes, reference_anchor_times
            )
            print(f"    Time warping completed. Real data now aligned to Simulation timeline.")

        all_data[obstacle_condition] = {
            'sim_times': sim_times,
            'sim_similarities': sim_similarities,
            'real_times': real_times,
            'real_similarities': real_similarities,
            'formation_changes': sim_formation_changes  # Use simulation as reference
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
    # Fixed y-axis for publication (allows comparison across papers)
    # 0.6 accommodates formation transitions (~0.35) with headroom
    common_ylim = (0, 0.6)

    # Second pass: create plots with common axis ranges
    for obstacle_condition in experiments.keys():
        # Create a separate figure for each condition
        fig, ax = plt.subplots(figsize=(14, 8))

        print(f"\n{'='*60}")
        print(f"Plotting: {obstacle_condition}")
        print(f"{'='*60}")

        # Get pre-processed data
        data = all_data[obstacle_condition]
        sim_times = data['sim_times']
        sim_similarities = data['sim_similarities']
        real_times = data['real_times']
        real_similarities = data['real_similarities']
        formation_changes = data['formation_changes']

        # Add shaded regions for formation transitions BEFORE plotting lines
        if formation_changes:
            # Add shaded regions for step1->step2 transitions (formation stabilizing)
            transition_label_added = False
            for i in range(len(formation_changes) - 1):
                current_formation = formation_changes[i]['type']
                next_formation = formation_changes[i + 1]['type']

                # Shade only when formation type is SAME (step1 -> step2 within same formation)
                # This is the period where formation is stabilizing after initial change
                is_step1_to_step2 = (current_formation == next_formation)

                if is_step1_to_step2:
                    # Shade the transition region (step1 -> step2)
                    start_time = formation_changes[i]['time']
                    end_time = formation_changes[i + 1]['time']
                    label = 'Formation transition' if not transition_label_added else ''
                    ax.axvspan(start_time, end_time, alpha=0.35, color=shade_color,
                              edgecolor='orange', linewidth=2.0, zorder=0, label=label)
                    if label:
                        transition_label_added = True

        # Plot simulation data
        if len(sim_times) > 0:
            ax.plot(sim_times, sim_similarities, color=sim_color,
                   linewidth=2.5, alpha=0.9, label='Simulation', zorder=3)

            # Print statistics
            mean_sim = np.mean(sim_similarities)
            max_sim = np.max(sim_similarities)
            final_sim = sim_similarities[-1] if len(sim_similarities) > 0 else 0

            print(f"  Simulation statistics:")
            print(f"    Mean similarity: {mean_sim:.6f}")
            print(f"    Max similarity: {max_sim:.6f}")
            print(f"    Final similarity: {final_sim:.6f}")
            print(f"    Duration: {sim_times[-1]:.2f} seconds")
            print(f"    Data points: {len(sim_times)}")

        # Plot real data
        if len(real_times) > 0:
            ax.plot(real_times, real_similarities, color=real_color,
                   linewidth=2.5, alpha=0.9, label='Real world', zorder=3)

            # Print statistics
            mean_sim = np.mean(real_similarities)
            max_sim = np.max(real_similarities)
            final_sim = real_similarities[-1] if len(real_similarities) > 0 else 0

            print(f"  Real statistics:")
            print(f"    Mean similarity: {mean_sim:.6f}")
            print(f"    Max similarity: {max_sim:.6f}")
            print(f"    Final similarity: {final_sim:.6f}")
            print(f"    Duration: {real_times[-1]:.2f} seconds")
            print(f"    Data points: {len(real_times)}")

        # Add vertical lines for formation changes
        if formation_changes:
            for fc in formation_changes:
                ax.axvline(x=fc['time'], color='#888888', linestyle='--',
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
        output_base = f'real_formation_shape_similarity_{condition_suffix}'

        # High-resolution PNG for general use
        plt.savefig(f'{output_base}.png', dpi=600, bbox_inches='tight', facecolor='white')

        # PDF for LaTeX/publication (vector format)
        plt.savefig(f'{output_base}.pdf', dpi=600, bbox_inches='tight', facecolor='white')

        print(f"\nFigures saved:")
        print(f"  - {output_base}.png (raster, 600 DPI)")
        print(f"  - {output_base}.pdf (vector, for LaTeX)")

        # Close the figure to free memory
        plt.close(fig)

    print(f"\n{'='*60}")
    print(f"All figures saved successfully!")
    print(f"{'='*60}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analyze real flight formation shape similarity')
    parser.add_argument('--mode', type=str, default='compare', choices=['compare', 'single'],
                       help='Mode: "compare" for Sim vs Real comparison, "single" for single experiment analysis')
    parser.add_argument('--px4_dir', type=str,
                       default='/home/lim/workspace/ros_ws/swarm-formation/logs/px4/1208_no_obs',
                       help='Directory containing PX4 ULG files (for single mode)')
    parser.add_argument('--jetson_dir', type=str,
                       default='/home/lim/workspace/ros_ws/swarm-formation/logs/jetson/real/jetson_simul_1208_no_obs',
                       help='Directory containing Jetson log files (for single mode)')
    parser.add_argument('--output_dir', type=str,
                       default='/home/lim/workspace/ros_ws/swarm-formation',
                       help='Output directory for plots (for single mode)')

    args = parser.parse_args()

    if args.mode == 'compare':
        # New comparison mode (default)
        main()
    else:
        # Legacy single experiment mode
        analyze_formation_quality(args.px4_dir, args.jetson_dir, args.output_dir)