import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re

# Read the CSV file
import glob

# Find CSV file with largest timestamp
files = glob.glob('imu_data_*.csv')
newest_file = max(files, key=lambda x: int(re.search(r'imu_data_(\d+)\.csv', x).group(1)) if re.search(r'imu_data_(\d+)\.csv', x) else 0)

print(f"Newest file: {newest_file}")

# Load the newest file
df = pd.read_csv(newest_file)
print(f"File contains {len(df)} rows")
if 'timestamp' in df.columns:
    print(f"Time range: {df['timestamp'].min()} to {df['timestamp'].max()}")
    print(f"That is {len(df) / (df['timestamp'].max() - df['timestamp'].min())} Hz")



# Calculate the average time between accelerometer value changes
def get_interval_between_changes(df, column):
    changes = df[column].diff().abs() > 0
    change_indices = df.index[changes].tolist()
    if len(change_indices) < 2:
        return 0
    
    time_at_changes = df.iloc[change_indices]['timestamp'].values
    intervals = np.diff(time_at_changes)
    return np.mean(intervals)

# Calculate intervals for each accelerometer axis
interval_x = get_interval_between_changes(df, 'acc_x')
interval_y = get_interval_between_changes(df, 'acc_y')
interval_z = get_interval_between_changes(df, 'acc_z')
avg_interval = np.mean([interval_x, interval_y, interval_z])

# Convert intervals to frequency (Hz)
freq_x = 1.0 / interval_x if interval_x > 0 else 0
freq_y = 1.0 / interval_y if interval_y > 0 else 0
freq_z = 1.0 / interval_z if interval_z > 0 else 0
avg_freq = 1.0 / avg_interval if avg_interval > 0 else 0

# Create a figure with 4 subplots
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
# kalman_filter_value = input("Enter the kalman filter value: ")
fig.suptitle(f'IMU Shaking around - Avg accelerometer change rate: {avg_freq:.2f} Hz\nX: {freq_x:.2f} Hz, Y: {freq_y:.2f} Hz, Z: {freq_z:.2f} Hz', fontsize=16)

# Function to plot data with breaks at timestamp discontinuities
def plot_with_breaks(ax, x, y, label, max_gap=0.1):
    # Calculate timestamp differences
    time_diffs = np.diff(x)
    # Find indices where there are large gaps in time
    break_indices = np.where(time_diffs > max_gap)[0]
    
    # If no breaks, plot normally
    if len(break_indices) == 0:
        ax.plot(x, y, label=label)
        return
    
    # Plot each continuous segment separately
    start_idx = 0
    for idx in break_indices:
        ax.plot(x[start_idx:idx+1], y[start_idx:idx+1], label=label if start_idx == 0 else "")
        start_idx = idx + 1
    
    # Plot the last segment
    if start_idx < len(x):
        ax.plot(x[start_idx:], y[start_idx:], label="" if start_idx > 0 else label)

# Plot accelerometer data
plot_with_breaks(ax1, df['timestamp'], df['acc_x'], 'X')
plot_with_breaks(ax1, df['timestamp'], df['acc_y'], 'Y')
plot_with_breaks(ax1, df['timestamp'], df['acc_z'], 'Z')
ax1.set_title('Accelerometer Data')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Acceleration (m/s²)')
ax1.legend()
ax1.grid(True)

# Plot gyroscope data with ±2 rad/s threshold
plot_with_breaks(ax2, df['timestamp'], df['gyro_x'], 'X')
plot_with_breaks(ax2, df['timestamp'], df['gyro_y'], 'Y')
plot_with_breaks(ax2, df['timestamp'], df['gyro_z'], 'Z')
ax2.axhline(y=2, color='r', linestyle='--', alpha=0.5)
ax2.axhline(y=-2, color='r', linestyle='--', alpha=0.5)
# ax2.fill_between(df['timestamp'], -2, 2, color='gray', alpha=0.2)
ax2.set_title('Gyroscope Data')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angular Velocity (rad/s)')
ax2.legend()
ax2.grid(True)

# Plot quaternion data
plot_with_breaks(ax4, df['timestamp'], df['quat_x'], 'X')
plot_with_breaks(ax4, df['timestamp'], df['quat_y'], 'Y')
plot_with_breaks(ax4, df['timestamp'], df['quat_z'], 'Z')
plot_with_breaks(ax4, df['timestamp'], df['quat_w'], 'W')
ax4.set_title('Quaternion')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Value')
ax4.legend()
ax4.grid(True)

# # Plot angle data
# plot_with_breaks(ax4, df['timestamp'], df['angle_x'], 'X')
# plot_with_breaks(ax4, df['timestamp'], df['angle_y'], 'Y')
# plot_with_breaks(ax4, df['timestamp'], df['angle_z'], 'Z')
# ax4.set_title('Angle Data')
# ax4.set_xlabel('Time (s)')
# ax4.set_ylabel('Angle (rad)')
# ax4.legend()
# ax4.grid(True)

# Plot XY accelerometer data
plot_with_breaks(ax3, df['timestamp'], df['acc_x'], 'X')
plot_with_breaks(ax3, df['timestamp'], df['acc_y'], 'Y')
ax3.set_title('XY Accelerometer Data')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Acceleration (m/s²)')
ax3.legend()
ax3.grid(True)

# Analyze time periods when gyroscope values are outside ±2 rad/s
# threshold = 2
# text_content = []
# for axis in ['x', 'y', 'z']:
#     gyro_col = f'gyro_{axis}'
#     outside_threshold = (df[gyro_col].abs() > threshold)
#     time_periods = []
#     start_time = None
    
#     for i, (time, is_outside) in enumerate(zip(df['timestamp'], outside_threshold)):
#         if is_outside and start_time is None:
#             start_time = time
#         elif not is_outside and start_time is not None:
#             time_periods.append((start_time, time))
#             start_time = None
    
#     # Handle case where the last point is outside threshold
#     if start_time is not None:
#         time_periods.append((start_time, df['timestamp'].iloc[-1]))
    
#     # Add text content for this axis
#     text_content.append(f"Gyroscope {axis.upper()} outside ±{threshold} rad/s:")
#     for start, end in time_periods:
#         text_content.append(f"  {start:.2f}s to {end:.2f}s ({end-start:.2f}s)")
#     text_content.append("")  # Add empty line between axes
        
#     # Add shaded regions for outside threshold periods
#     for start, end in time_periods:
#         mask = (df['timestamp'] >= start) & (df['timestamp'] <= end)
#         ax2.fill_between(df['timestamp'][mask], 
#                         df[gyro_col][mask], 
#                         np.where(df[gyro_col][mask] > 0, 2, -2),
#                         color='red', alpha=0.2)

# # Add text box to the Euler angles position
# text_str = '\n'.join(text_content)
# props = dict(boxstyle='round', facecolor='white', alpha=0.8)
# ax3.text(0.5, 0.5, text_str, fontsize=12, ha='center', va='center',
#          transform=ax3.transAxes, bbox=props)
# ax3.axis('off')  # Turn off the axis

# Adjust layout and save the plot
plt.tight_layout()
import time
plt.savefig(f'{int(time.time())}_imu_plots.png')
plt.show() 
