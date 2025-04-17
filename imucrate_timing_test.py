import pandas as pd
import matplotlib.pyplot as plt
import glob
import re

# Find CSV file with largest timestamp
files = glob.glob('imu_data_*.csv')
newest_file = max(files, key=lambda x: int(re.search(r'imu_data_(\d+)\.csv', x).group(1)) if re.search(r'imu_data_(\d+)\.csv', x) else 0)

print(f"Newest file: {newest_file}")

# Load the newest file
df = pd.read_csv(newest_file)
print(f"File contains {len(df)} rows")
if 'timestamp' in df.columns:
    print(f"Time range: {df['timestamp'].min()} to {df['timestamp'].max()}")

# 1) Plot accelerometer data
plt.figure()
plt.plot(df['timestamp'], df['acc_x'], label='acc_x')
plt.plot(df['timestamp'], df['acc_y'], label='acc_y')
plt.plot(df['timestamp'], df['acc_z'], label='acc_z')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.title('Accelerometer vs Time')
plt.legend()
plt.tight_layout()
plt.show()

# 2) Plot gyroscope data
plt.figure()
plt.plot(df['timestamp'], df['gyro_x'], label='gyro_x')
plt.plot(df['timestamp'], df['gyro_y'], label='gyro_y')
plt.plot(df['timestamp'], df['gyro_z'], label='gyro_z')
plt.xlabel('Time (s)')
plt.ylabel('Angular velocity (°/s)')
plt.title('Gyroscope vs Time')
plt.legend()
plt.tight_layout()
plt.show()

# 3) Detect when any value actually changes
sensor_cols = [
    'acc_x','acc_y','acc_z',
    'gyro_x','gyro_y','gyro_z',
    'angle_x','angle_y','angle_z',
    'quat_x','quat_y','quat_z','quat_w'
]
# Boolean series: True where any sensor differs from previous
changes = df[sensor_cols].diff().abs().sum(axis=1) > 0
update_times = df.loc[changes, 'timestamp'].reset_index(drop=True)

# 4) Compute intervals and frequencies
intervals = update_times.diff().dropna()
frequencies = 1.0 / intervals

# Print intervals and frequencies
print("Update intervals (s):")
print(intervals.values)
print("\nFrequencies (Hz):")
print(frequencies.values)
print(f"\nAverage interval: {intervals.mean():.6f} s")
print(f"Average frequency: {frequencies.mean():.2f} Hz")

