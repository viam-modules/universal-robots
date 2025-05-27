import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def plot_data(df, time_col, joint_prefix, ylabel, title, filename, convert_func=None, count=6):
    plt.figure(figsize=(20, 16))
    for i in range(count):
        col = f'{joint_prefix}{i}'
        data = convert_func(df[col]) if convert_func else df[col]
        plt.plot(df[time_col], data, label=col)
    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()

if len(sys.argv) != 2:
    print(f"Usage: python {sys.argv[0]} path/to/file.csv")
    sys.exit(1)

csv_path = sys.argv[1]

if not os.path.isfile(csv_path):
    print(f"Error: File '{csv_path}' does not exist.")
    sys.exit(1)

df = pd.read_csv(csv_path)

# Originally, time in this specific csv is relative
# we need to add things up to get absolute time
df['time_cumulative'] = df['t(s)'].cumsum()

# Need to derive acceleration from velocity
time = df['time_cumulative'].to_numpy()
for i in range(6):
    # Note: `f` here is a formatted string
    vel_col = f'v{i}'
    acc_col = f'a{i}'
    df[acc_col] = np.gradient(df[vel_col], time)

# Plot joint positions (radians)
plot_data(
    df,
    'time_cumulative',
    'j',
    'Position (rad)',
    'Desired Joint Positions over Time (rad)',
    'desired_joint_positions_rad.png'
)

# Plot joint positions (degrees)
plot_data(
    df,
    'time_cumulative',
    'j',
    'Position (deg)',
    'Desired Joint Positions over Time (deg)',
    'desired_joint_positions_deg.png',
    convert_func=np.degrees
)

# Plot joint velocities (radians/s) ---
plot_data(
    df,
    'time_cumulative',
    'v',
    'Velocity (rad/s)',
    'Desired Joint Velocities over Time (rad/s)',
    'desired_joint_velocities_rad.png'
)

# Plot joint velocities (degrees/s) ---
plot_data(
    df,
    'time_cumulative',
    'v',
    'Velocity (deg/s)',
    'Desired Joint Velocities over Time (deg/s)',
    'desired_joint_velocities_deg.png',
    convert_func=np.degrees
)

# Plot joint accelerations (radians/s^2)
plot_data(
    df,
    'time_cumulative',
    'a',
    'Acceleration (rad/s^2)',
    'Desired Joint Accelerations over Time (rad/s^2)',
    'desired_joint_accelerations_rad.png'
)

# Plot joint accelerations (degrees/s^2)
plot_data(
    df,
    'time_cumulative',
    'a',
    'Acceleration (deg/s^2)',
    'Desired Joint Accelerations over Time (deg/s^2)',
    'desired_joint_accelerations_deg.png',
    convert_func=np.degrees
)
