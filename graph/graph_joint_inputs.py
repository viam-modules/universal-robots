import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import json

def plot_joint_positions(time_of_traj_gen, traj_gen_joint_information, motion_time, motions_positions, joint_idx, filename):
    plt.figure(figsize=(20, 16))
    plt.plot(motion_time, motions_positions, label='motion plan', linestyle='--')
    plt.plot(time_of_traj_gen, traj_gen_joint_information, label='traj gen', linewidth=2)
    plt.title(f'Joint {joint_idx} Position over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
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

traj_gen = config.get("trajectory_csv")
motion = config.get("waypoints_csv")

if not traj_gen or not motion:
    print("Error: 'trajectory_csv' or 'waypoints_csv' not found in config.json.")
    sys.exit(1)

if not os.path.isfile(traj_gen):
    print(f"Error: File '{traj_gen}' does not exist.")
    sys.exit(1)

if not os.path.isfile(motion):
    print(f"Error: File '{motion}' does not exist.")
    sys.exit(1)

# load trajectory generation data
df_traj_gen = pd.read_csv(traj_gen)
# time in this file is relative so we need to add it all up
df_traj_gen['time_cumulative'] = df_traj_gen['t(s)'].cumsum()
time_of_traj_gen = df_traj_gen['time_cumulative'].to_numpy()

# load motion data (inputs passed into MoveThroughJointPositions)
motion_data = np.loadtxt(motion, delimiter=',')

# induce a metric of time into the motion data (linearly spaced to match desired time span)
motion_time = np.linspace(time_of_traj_gen[0], time_of_traj_gen[-1], motion_data.shape[0])

for i in range(6):
    traj_gen_joint_information = df_traj_gen[f'j{i}'].to_numpy()
    motions_positions = motion_data[:, i]
    filename = f'joint_{i}_position_comparison.png'
    plot_joint_positions(time_of_traj_gen, traj_gen_joint_information, motion_time, motions_positions, i, filename)

