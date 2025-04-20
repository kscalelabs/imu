import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file
df = pd.read_csv('imu_data.csv')

# Enhanced frequency analysis with duplicate detection
# Create a combined feature to detect any change in sensor readings
df['data_hash'] = df[['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']].astype(str).sum(axis=1)

# Detect duplicates and count them
df['is_duplicate'] = df['data_hash'].shift() == df['data_hash']
df.loc[0, 'is_duplicate'] = False  # First row is never a duplicate

# Count statistics
total_readings = len(df)
total_duplicates = df['is_duplicate'].sum()
duplicate_percent = (total_duplicates / total_readings) * 100 if total_readings > 0 else 0
runtime = df['timestamp'].max() - df['timestamp'].min()

# Find indices where data actually changes (non-duplicates)
data_change_indices = df.index[~df['is_duplicate']].tolist()

# Calculate frequencies
raw_hz = total_readings / runtime if runtime > 0 else 0
effective_hz = (total_readings - total_duplicates) / runtime if runtime > 0 else 0

# Calculate average time difference between unique readings
if len(data_change_indices) > 1:
    real_time_diff = np.diff([df.iloc[i]['timestamp'] for i in data_change_indices])
    avg_time_diff = np.mean(real_time_diff)
    unique_update_hz = 1 / avg_time_diff if avg_time_diff > 0 else 0
else:
    unique_update_hz = 0

# Print frequency analysis summary
print(f"Stats: {total_duplicates} duplicates/{total_readings} total readings ({duplicate_percent:.1f}%) over {runtime:.1f}s")
print(f"Effective rate: {effective_hz:.1f} Hz (Raw: {raw_hz:.1f} Hz)")
print(f"Average frequency between unique updates: {unique_update_hz:.2f} Hz")

# Create a figure with 4 subplots
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
title_str = input("Enter file name: ")
fig.suptitle(f'IMU Analysis {title_str} - Raw: {raw_hz:.1f} Hz, Effective: {effective_hz:.1f} Hz, Duplicates: {duplicate_percent:.1f}%', fontsize=16)

# Plot accelerometer data
ax1.plot(df['timestamp'], df['acc_x'], label='X')
ax1.plot(df['timestamp'], df['acc_y'], label='Y')
ax1.plot(df['timestamp'], df['acc_z'], label='Z')
ax1.set_title('Accelerometer Data')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Acceleration (m/s²)')
ax1.legend()
ax1.grid(True)

# Plot gyroscope data with ±2 rad/s threshold
ax2.plot(df['timestamp'], df['gyro_x'], label='X')
ax2.plot(df['timestamp'], df['gyro_y'], label='Y')
ax2.plot(df['timestamp'], df['gyro_z'], label='Z')
ax2.axhline(y=2, color='r', linestyle='--', alpha=0.5)
ax2.axhline(y=-2, color='r', linestyle='--', alpha=0.5)
# ax2.fill_between(df['timestamp'], -2, 2, color='gray', alpha=0.2)
ax2.set_title('Gyroscope Data')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angular Velocity (rad/s)')
ax2.legend()
ax2.grid(True)

# Plot quaternion data
ax4.plot(df['timestamp'], df['quat_x'], label='X')
ax4.plot(df['timestamp'], df['quat_y'], label='Y')
ax4.plot(df['timestamp'], df['quat_z'], label='Z')
ax4.plot(df['timestamp'], df['quat_w'], label='W')
ax4.set_title('Quaternion')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Value')
ax4.legend()
ax4.grid(True)

# Plot XY accelerometer data
ax3.plot(df['timestamp'], df['acc_x'], label='X')
ax3.plot(df['timestamp'], df['acc_y'], label='Y')
ax3.set_title('XY Accelerometer Data')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Acceleration (m/s²)')
ax3.legend()
ax3.grid(True)

# Adjust layout and save the plot
plt.tight_layout()
plt.savefig(f'{title_str}_imu_plots.png')
plt.show() 