import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import numpy as np

def plot_data(df, time_col, columns, ylabel, title, filename, convert_func=None):
    plt.figure(figsize=(20, 16))
    for col in columns:
        data = convert_func(df[col]) if convert_func else df[col]
        plt.plot(df[time_col], data, label=col)
    plt.title(title)
    plt.xlabel("Time (s)")
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

# Originally time is in unix which is milliseconds, we convert to seconds
df['time_s'] = df['time_ms'] / 1000.0

# Joint position columns (radians)
joint_columns_rad = [f'joint_{i}_rad' for i in range(6)]
# Joint position columns (degrees) -- at this point still needs to be computed
joint_columns_deg = [f'joint_{i}_deg' for i in range(6)]

# Compute degree positions and velocities
for i, joint_rad in enumerate(joint_columns_rad):
    joint_deg = joint_columns_deg[i]
    # Note: `f` here is a formatted string
    vel_rad = f'joint_{i}_velocity_rad'
    vel_deg = f'joint_{i}_velocity_deg'
    accel_rad = f'joint_{i}_acceleration_rad'
    accel_deg = f'joint_{i}_acceleration_deg'

    # Convert positions to degrees
    df[joint_deg] = np.degrees(df[joint_rad])

    # Calculate velocities in rad/s
    df[vel_rad] = df[joint_rad].diff() / df['time_s'].diff()
    # Calculate velocities in deg/s
    df[vel_deg] = np.degrees(df[vel_rad])
    
    # Calculate accelerations in rad/s^2
    df[accel_rad] = df[vel_rad].diff() / df['time_s'].diff()
    # Calculate accelerations in deg/s^2
    df[accel_deg] = np.degrees(df[accel_rad])


# Plot joint positions (radians)
plot_data(
    df,
    'time_s',
    joint_columns_rad,
    "Position (rad)",
    "Actual Joint Positions Over Time (rad)",
    "actual_joint_positions_rad.png"
)

# Plot joint positions (degrees)
plot_data(
    df,
    'time_s',
    joint_columns_deg,
    "Position (deg)",
    "Actual Joint Positions Over Time (deg)",
    "actual_joint_positions_deg.png"
)

# Plot joint velocities (radians/s)
velocity_rad_cols = [f'joint_{i}_velocity_rad' for i in range(6)]
plot_data(
    df,
    'time_s',
    velocity_rad_cols,
    "Velocity (rad/s)",
    "Actual Joint Velocities Over Time (rad/s)",
    "actual_joint_velocities_rad.png"
)

# Plot joint velocities (degrees/s)
velocity_deg_cols = [f'joint_{i}_velocity_deg' for i in range(6)]
plot_data(
    df,
    'time_s',
    velocity_deg_cols,
    "Velocity (deg/s)",
    "Actual Joint Velocities Over Time (deg/s)",
    "actual_joint_velocities_deg.png"
)

# Plot joint acclerations (radians/s^2)
acceleration_rad_cols = [f'joint_{i}_acceleration_rad' for i in range(6)]
plot_data(
    df,
    'time_s',
    acceleration_rad_cols,
    "Acceleration (rad/s)",
    "Actual Joint Accelerations Over Time (rad/s)",
    "actual_joint_accelerations_rad.png"
)

# Plot joint acclerations (degrees/s^2)
acceleration_deg_cols = [f'joint_{i}_acceleration_deg' for i in range(6)]
plot_data(
    df,
    'time_s',
    acceleration_deg_cols,
    "Acceleration (deg/s)",
    "Actual Joint Accelerations Over Time (deg/s)",
    "actual_joint_accelerations_deg.png"
)
