import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

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

if len(sys.argv) != 3:
    print(f"Usage: python {sys.argv[0]} path/to/trajectory.csv path/to/waypoints.csv")
    sys.exit(1)

# the first argument is a path leading to *trajectory.csv <- this is what trajectory generation returns 
# the second argument is a path leading to *waypoints.csv <- this is what motion planning returns
traj_gen = sys.argv[1]
motion = sys.argv[2]

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

