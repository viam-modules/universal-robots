import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import json

def expand_path(path):
    return os.path.expanduser(path)

def load_config(path="config.json"):
    if not os.path.isfile(path):
        print(f"Error: Config file '{path}' does not exist.")
        sys.exit(1)
    with open(path, 'r') as f:
        return json.load(f)

def check_file_exists(path, label):
    if not os.path.isfile(path):
        print(f"Error: {label} file '{path}' does not exist.")
        sys.exit(1)

def compute_cumulative_time(df, time_col='t(s)'):
    df['time_cumulative'] = df[time_col].cumsum()
    return df['time_cumulative'].to_numpy()

def plot_comparison_df(time_ref, data_ref, time_cmp, data_cmp, labels, ylabel, filename, use_markers=False):
    plt.figure(figsize=(20, 16))
    df_ref = pd.DataFrame({ 'time': time_ref, 'trajectory': data_ref })
    df_cmp = pd.DataFrame({ 'time': time_cmp, 'waypoints': data_cmp })

    ax = df_ref.plot(x='time', y='trajectory', label=labels[0], linewidth=2)
    df_cmp.plot(x='time', y='waypoints', label=labels[1], ax=ax,
                linestyle='--', marker='o' if use_markers else None, markersize=4)

    plt.title(f'{labels[0]} vs {labels[1]} over Time')
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()

def main():
    config = load_config()

    # plot joints
    if 'trajectory_csv' in config and 'waypoints_csv' in config:
        traj_gen_path = expand_path(config['trajectory_csv'])
        motion_path = expand_path(config['waypoints_csv'])
        check_file_exists(traj_gen_path, 'Trajectory CSV')
        check_file_exists(motion_path, 'Waypoints CSV')

        df_traj_gen = pd.read_csv(traj_gen_path)
        time_traj_gen = compute_cumulative_time(df_traj_gen)

        motion_data = np.loadtxt(motion_path, delimiter=',')
        motion_time = np.linspace(time_traj_gen[0], time_traj_gen[-1], motion_data.shape[0])

        for i in range(6):
            plot_comparison_df(
                time_traj_gen,
                df_traj_gen[f'j{i}'],
                motion_time,
                motion_data[:, i],
                [f'Joint j{i} Trajectory', 'Motion Plan'],
                'Position (rad)',
                f'joint_{i}_position_comparison.png'
            )

    # plot poses
    if 'output_trajectory_poses' in config and 'output_waypoint_poses' in config:
        traj_pose_path = expand_path(config['output_trajectory_poses'])
        waypt_pose_path = expand_path(config['output_waypoint_poses'])
        check_file_exists(traj_pose_path, 'Trajectory Pose CSV')
        check_file_exists(waypt_pose_path, 'Waypoint Pose CSV')

        df_traj_pose = pd.read_csv(traj_pose_path)
        time_traj_pose = compute_cumulative_time(df_traj_pose)

        df_waypt_pose = pd.read_csv(waypt_pose_path)
        time_waypt_pose = np.linspace(time_traj_pose[0], time_traj_pose[-1], df_waypt_pose.shape[0])

        for axis in ['x', 'y', 'z']:
            interp_data = np.interp(time_traj_pose, time_waypt_pose, df_waypt_pose[axis].values)
            plot_comparison_df(
                time_traj_pose,
                df_traj_pose[axis],
                time_traj_pose,
                interp_data,
                [f'{axis.upper()} Trajectory', f'{axis.upper()} Waypoints'],
                f'{axis.upper()} Position (m)',
                f'{axis}_comparison.png',
                use_markers=True
            )


if __name__ == '__main__':
    main()
