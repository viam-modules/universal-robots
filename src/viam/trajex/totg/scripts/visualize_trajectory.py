#!/usr/bin/env python3
"""
Visualize trajectory generation from JSON output.

Usage:
    python scripts/visualize_trajectory.py trajectory.json
"""

import json
import sys
import matplotlib.pyplot as plt
import numpy as np

# Visualization parameters
LIMIT_CURVE_MARGIN = 1.15
MAX_Y_SCALE_FACTOR = 2.5
SWITCHING_POINT_SIZE = 120
SWITCHING_POINT_ALPHA = 0.7
SWITCHING_POINT_LINE_WIDTH = 1
LIMIT_MARKER_SIZE = 10


def load_trajectory(filename):
    """Load trajectory JSON file with validation."""
    try:
        with open(filename) as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: File not found: {filename}", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON: {e}", file=sys.stderr)
        sys.exit(1)

    # Validate required top-level keys
    required = ['metadata', 'integration_points', 'events']
    missing = [k for k in required if k not in data]
    if missing:
        print(f"Error: Missing required keys: {missing}", file=sys.stderr)
        sys.exit(1)

    return data


def plot_phase_plane(data, ax):
    """Plot phase plane trajectory (s, s_dot) with limit curves."""
    # Main trajectory (green like the paper)
    s = np.array([float(x) for x in data['integration_points']['s']])
    s_dot = np.array([float(x) for x in data['integration_points']['s_dot']])

    ax.plot(s, s_dot, 'g-', linewidth=2, label='Trajectory', zorder=3)

    # Limit curves with discontinuity handling
    s_dot_max_acc = data['integration_points']['s_dot_max_acc']
    s_dot_max_vel = data['integration_points']['s_dot_max_vel']

    # Calculate y-axis limits ensuring trajectory visibility:
    # - Trajectory should occupy at least 60% of vertical space
    # - If limits are very high (e.g., nearly infinite), cap y_max to avoid
    #   squashing the trajectory into an unreadable band at the bottom
    # - Use MAX_Y_SCALE_FACTOR as a heuristic balance
    max_traj_velocity = max(s_dot)

    # Collect all limit curve values
    limit_values = []
    for val in s_dot_max_acc:
        if val is not None:
            limit_values.append(float(val))
    for val in s_dot_max_vel:
        if val is not None:
            limit_values.append(float(val))

    if limit_values:
        max_limit = max(limit_values)
        y_max = min(max_limit * LIMIT_CURVE_MARGIN, max_traj_velocity * MAX_Y_SCALE_FACTOR)
    else:
        y_max = max_traj_velocity * 1.3

    # Add buffer below trajectory
    y_min = -0.05 * max_traj_velocity

    # Acceleration limit curve (red - primary boundary)
    # Track transitions to show vertical bars
    prev_was_inf = True
    prev_s = None
    prev_val = None
    for i, val in enumerate(s_dot_max_acc):
        curr_s = float(s[i])
        if val is not None:
            curr_val = float(val)
            # Show vertical drop when transitioning from infinite to finite
            if prev_was_inf and i > 0:
                ax.plot([curr_s, curr_s], [y_max, curr_val], 'r-',
                       linewidth=1, alpha=0.5, zorder=1)
            prev_was_inf = False
            prev_s = curr_s
            prev_val = curr_val
        else:
            # Show vertical rise when transitioning from finite to infinite
            if not prev_was_inf and prev_s is not None and prev_val is not None:
                ax.plot([prev_s, prev_s], [prev_val, y_max], 'r-',
                       linewidth=1, alpha=0.5, zorder=1)
            prev_was_inf = True

    # Plot the actual limit curve segments
    segments_acc = []
    current_segment_s = []
    current_segment_v = []
    for i, val in enumerate(s_dot_max_acc):
        if val is not None:
            current_segment_s.append(float(s[i]))
            current_segment_v.append(float(val))
        else:
            if current_segment_s:
                segments_acc.append((current_segment_s, current_segment_v))
                current_segment_s = []
                current_segment_v = []
    if current_segment_s:
        segments_acc.append((current_segment_s, current_segment_v))

    for i, (seg_s, seg_v) in enumerate(segments_acc):
        label = 'Acceleration Limit' if i == 0 else None
        ax.plot(seg_s, seg_v, 'r-', linewidth=1.5, label=label, zorder=2)

    # Velocity limit curve (orange - secondary boundary)
    prev_was_inf = True
    prev_s = None
    prev_val = None
    for i, val in enumerate(s_dot_max_vel):
        curr_s = float(s[i])
        if val is not None:
            curr_val = float(val)
            # Show vertical drop when transitioning from infinite to finite
            if prev_was_inf and i > 0:
                ax.plot([curr_s, curr_s], [y_max, curr_val], 'orange',
                       linewidth=1, alpha=0.5, zorder=1)
            prev_was_inf = False
            prev_s = curr_s
            prev_val = curr_val
        else:
            # Show vertical rise when transitioning from finite to infinite
            if not prev_was_inf and prev_s is not None and prev_val is not None:
                ax.plot([prev_s, prev_s], [prev_val, y_max], 'orange',
                       linewidth=1, alpha=0.5, zorder=1)
            prev_was_inf = True

    segments_vel = []
    current_segment_s = []
    current_segment_v = []
    for i, val in enumerate(s_dot_max_vel):
        if val is not None:
            current_segment_s.append(float(s[i]))
            current_segment_v.append(float(val))
        else:
            if current_segment_s:
                segments_vel.append((current_segment_s, current_segment_v))
                current_segment_s = []
                current_segment_v = []
    if current_segment_s:
        segments_vel.append((current_segment_s, current_segment_v))

    for i, (seg_s, seg_v) in enumerate(segments_vel):
        label = 'Velocity Limit' if i == 0 else None
        ax.plot(seg_s, seg_v, 'orange', linewidth=1.5, label=label, zorder=2)

    # Interior switching points (not start/end)
    # Map kind to marker style
    kind_markers = {
        'k_discontinuous_curvature': ('s', 'Curvature Discontinuity'),
        'k_nondifferentiable_extremum': ('D', 'Non-diff Extremum'),
        'k_velocity_escape': ('v', 'Velocity Escape'),
        'k_discontinuous_velocity_limit': ('p', 'Velocity Limit Discontinuity'),
    }

    kind_seen = set()
    for event in data['events']['backward_starts']:
        kind = event['kind']
        # Skip path begin and end
        if kind in ('k_path_begin', 'k_path_end'):
            continue

        marker, label_text = kind_markers.get(kind, ('^', kind))
        label = label_text if kind not in kind_seen else None
        kind_seen.add(kind)

        # Hollow markers with transparency so they don't obscure the graph
        ax.scatter(float(event['s']), float(event['s_dot']),
                  marker=marker, s=SWITCHING_POINT_SIZE, facecolors='none', edgecolors='blue',
                  alpha=SWITCHING_POINT_ALPHA, linewidths=SWITCHING_POINT_LINE_WIDTH,
                  label=label, zorder=4)

    # Pruned points from splices (dashed green)
    pruned_shown = False
    for splice in data['events']['splices']:
        if 'pruned_points' in splice:
            pruned = splice['pruned_points']
            s_pruned = np.array([float(x) for x in pruned['s']])
            s_dot_pruned = np.array([float(x) for x in pruned['s_dot']])
            label = 'Pruned (replaced)' if not pruned_shown else None
            ax.plot(s_pruned, s_dot_pruned, 'g--', alpha=0.4,
                   linewidth=1.5, label=label, zorder=2.5)
            pruned_shown = True

    # Limit hits (black 'x' markers)
    if data['events']['limit_hits']:
        limit_hit_s = [float(event['s']) for event in data['events']['limit_hits']]
        limit_hit_s_dot = [float(event['s_dot']) for event in data['events']['limit_hits']]
        ax.scatter(limit_hit_s, limit_hit_s_dot, marker='x', s=100,
                   color='black', linewidths=2, label='Limit Hits', zorder=5)

    ax.set_xlabel('Arc Length s')
    ax.set_ylabel('Path Velocity s_dot')
    ax.set_title('Phase Plane Trajectory')
    ax.grid(True, alpha=0.3)
    # Set y-axis limits with buffer below and trajectory visibility above
    ax.set_ylim(y_min, y_max)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=3, frameon=True)


def plot_joint_trajectories(data, ax):
    """Plot joint-space position trajectories over time."""
    time = np.array([float(x) for x in data['integration_points']['time']])
    configs = data['integration_points']['configuration']
    dof = len(configs[0])

    for joint_idx in range(dof):
        positions = np.array([float(configs[i][joint_idx]) for i in range(len(configs))])
        ax.plot(time, positions, label=f'Joint {joint_idx}', linewidth=1)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (rad)')
    ax.set_title('Joint Trajectories')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')


def plot_joint_velocities(data, ax):
    """Plot joint-space velocity trajectories over time with limit markers."""
    time = np.array([float(x) for x in data['integration_points']['time']])
    velocities = data['integration_points']['velocity']
    dof = len(velocities[0])
    max_velocity = data['metadata']['max_velocity']

    # Store lines and their data for marker placement
    lines_data = []
    for joint_idx in range(dof):
        vels = np.array([float(velocities[i][joint_idx]) for i in range(len(velocities))])
        line, = ax.plot(time, vels, label=f'Joint {joint_idx}', linewidth=1.5)
        lines_data.append((line, vels, float(max_velocity[joint_idx])))

    # Add limit markers only if trajectory approaches the limit (within 85%)
    for line, vels, limit in lines_data:
        max_vel_reached = np.max(np.abs(vels))
        if max_vel_reached >= 0.85 * limit:
            color = line.get_color()
            ax.plot(0, limit, marker='>', color=color, markersize=LIMIT_MARKER_SIZE,
                    transform=ax.get_yaxis_transform(), clip_on=False, zorder=10)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (rad/s)')
    ax.set_title('Joint Velocities')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')


def plot_joint_accelerations(data, ax):
    """Plot joint-space acceleration trajectories over time with limit markers."""
    time = np.array([float(x) for x in data['integration_points']['time']])
    accelerations = data['integration_points']['acceleration']
    dof = len(accelerations[0])
    max_acceleration = data['metadata']['max_acceleration']

    # Store lines and their data for marker placement
    lines_data = []
    for joint_idx in range(dof):
        accels = np.array([float(accelerations[i][joint_idx]) for i in range(len(accelerations))])
        line, = ax.plot(time, accels, label=f'Joint {joint_idx}', linewidth=1.5)
        lines_data.append((line, accels, float(max_acceleration[joint_idx])))

    # Add limit markers only if trajectory approaches the limit (within 85%)
    for line, accels, limit in lines_data:
        max_accel_reached = np.max(np.abs(accels))
        if max_accel_reached >= 0.85 * limit:
            color = line.get_color()
            ax.plot(0, limit, marker='>', color=color, markersize=LIMIT_MARKER_SIZE,
                    transform=ax.get_yaxis_transform(), clip_on=False, zorder=10)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (rad/s²)')
    ax.set_title('Joint Accelerations')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')


def plot_arc_length_vs_time(data, ax):
    """Plot arc length progression over time."""
    time = np.array([float(x) for x in data['integration_points']['time']])
    s = np.array([float(x) for x in data['integration_points']['s']])

    ax.plot(time, s, 'b-', linewidth=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Arc Length s')
    ax.set_title('Arc Length vs Time')
    ax.grid(True, alpha=0.3)


def main():
    if len(sys.argv) < 2:
        print("Usage: python visualize_trajectory.py <trajectory.json>")
        sys.exit(1)

    filename = sys.argv[1]
    data = load_trajectory(filename)

    # Create figure with 3-row layout (taller to give phase plane more vertical space)
    fig = plt.figure(figsize=(14, 18))
    gs = fig.add_gridspec(3, 2)

    # Row 1: Phase plane spans both columns
    ax_phase = fig.add_subplot(gs[0, :])
    plot_phase_plane(data, ax_phase)

    # Row 2: Velocities and accelerations (both have limit markers)
    ax_vel = fig.add_subplot(gs[1, 0])
    plot_joint_velocities(data, ax_vel)

    ax_accel = fig.add_subplot(gs[1, 1])
    plot_joint_accelerations(data, ax_accel)

    # Row 3: Positions and arc length
    ax_pos = fig.add_subplot(gs[2, 0])
    plot_joint_trajectories(data, ax_pos)

    ax_arc = fig.add_subplot(gs[2, 1])
    plot_arc_length_vs_time(data, ax_arc)

    plt.tight_layout(rect=[0, 0.03, 1, 1])  # Leave space for legend at bottom
    plt.show()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
