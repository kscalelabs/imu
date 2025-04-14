import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file
df = pd.read_csv('imu_data.csv')

# Create a figure with 4 subplots
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
acc_filter_value = input("Enter the acc filter value: ")
fig.suptitle(f'IMU static on table with AccFilt: {acc_filter_value}', fontsize=16)

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
plt.savefig(f'{acc_filter_value}_imu_plots.png')
plt.show() 