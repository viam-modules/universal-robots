import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import json

def plot_axis(time_traj, traj_values, time_waypt, waypt_values, label, filename):
    plt.figure(figsize=(20, 16))
    plt.plot(time_waypt, waypt_values, label='waypoints (interpolated)', linestyle='--', marker='o', markersize=4)
    plt.plot(time_traj, traj_values, label='trajectory', linewidth=2)
    plt.title(f'{label.upper()} Position over Time')
    plt.xlabel('Time (s)')
    plt.ylabel(f'{label.upper()} Position (m)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()


config_path = "config.json"
if not os.path.isfile(config_path):
    print(f"Error: Config file '{config_path}' does not exist.")
    sys.exit(1)

with open(config_path, 'r') as f:
    config = json.load(f)

traj_csv = config.get("output_trajectory_poses")
waypt_csv = config.get("output_waypoint_poses")

if not traj_csv or not waypt_csv:
    print("Error: 'trajectory_csv' or 'waypoints_csv' not found in config.json.")
    sys.exit(1)

if not os.path.isfile(traj_csv):
    print(f"Error: File '{traj_csv}' does not exist.")
    sys.exit(1)

if not os.path.isfile(waypt_csv):
    print(f"Error: File '{waypt_csv}' does not exist.")
    sys.exit(1)


df_traj = pd.read_csv(traj_csv)
df_traj['time_cumulative'] = df_traj['t(s)'].cumsum()
time_traj = df_traj['time_cumulative'].to_numpy()


df_waypt = pd.read_csv(waypt_csv)
time_waypt = np.linspace(time_traj[0], time_traj[-1], df_waypt.shape[0])


x_interp = np.interp(time_traj, time_waypt, df_waypt['x'].values)
y_interp = np.interp(time_traj, time_waypt, df_waypt['y'].values)
z_interp = np.interp(time_traj, time_waypt, df_waypt['z'].values)

output_dir = os.path.dirname(os.path.abspath(__file__))

plot_axis(time_traj, df_traj['x'].values, time_traj, x_interp, 'x', os.path.join(output_dir, 'x_comparison.png'))
plot_axis(time_traj, df_traj['y'].values, time_traj, y_interp, 'y', os.path.join(output_dir, 'y_comparison.png'))
plot_axis(time_traj, df_traj['z'].values, time_traj, z_interp, 'z', os.path.join(output_dir, 'z_comparison.png'))
