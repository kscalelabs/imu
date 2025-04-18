import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse
from scipy.spatial.transform import Rotation

def calculate_accel_frequency(df):
    """
    Calculate the frequency of accelerometer data by measuring time differences
    between consecutive unique data points.
    
    Parameters:
    -----------
    df : pandas.DataFrame
        DataFrame containing accelerometer data with timestamp column
    
    Returns:
    --------
    pandas.DataFrame
        DataFrame with time and frequency columns
    """
    # Check if accelerometer data exists
    if not all(col in df.columns for col in ['accel_x', 'accel_y', 'accel_z']):
        return None
    
    # Create a copy with only timestamp and accelerometer data
    accel_df = df[['timestamp', 'time', 'accel_x', 'accel_y', 'accel_z']].copy()
    
    # Create a combined value to detect changes in any axis
    accel_df['accel_combined'] = accel_df['accel_x'].astype(str) + '_' + \
                                 accel_df['accel_y'].astype(str) + '_' + \
                                 accel_df['accel_z'].astype(str)
    
    # Find rows where data changes (is different from previous row)
    accel_df['is_different'] = accel_df['accel_combined'].shift() != accel_df['accel_combined']
    
    # Filter to keep only rows where data changed
    unique_data_points = accel_df[accel_df['is_different']].copy()
    
    if len(unique_data_points) <= 1:
        return None
    
    # Calculate time difference between consecutive unique points
    unique_data_points['dt'] = unique_data_points['timestamp'].diff()
    
    # Calculate frequency (1/dt) in Hz
    unique_data_points['freq'] = 1 / unique_data_points['dt']
    
    # Remove the first row which has NaN dt and freq
    unique_data_points = unique_data_points.dropna(subset=['dt'])
    
    # Return time and frequency columns
    return unique_data_points[['time', 'freq']]

def plot_imu_data(csv_file='imu_data.csv', output_dir='plots', fields_to_plot=None, title=None):
    """
    Plot IMU data from CSV file and save as PNG files
    
    Parameters:
    -----------
    csv_file : str
        Path to the CSV file containing IMU data
    output_dir : str
        Directory to save the output PNG files
    fields_to_plot : dict
        Dictionary of data fields to plot with boolean values
        Example: {'gyro': True, 'accel': True, 'quat': True, 'euler': True, 'gravity': True, 'lin_accel': False, 'mag': True}
        If None, all available fields will be plotted
    title : str
        Optional title to be applied to all plots
    """
    # Load data
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"File {csv_file} not found.")
        return
    
    # Modify output directory to include title if provided
    if title:
        # Replace spaces with underscores for folder name
        title_folder = title.replace(' ', '_')
        output_dir = f"{title_folder}_plots"
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Convert timestamp to relative time in seconds
    df['time'] = df['timestamp'] - df['timestamp'].iloc[0]
    
    # Check available fields in dataset
    has_gyro = all(col in df.columns for col in ['gyro_x', 'gyro_y', 'gyro_z'])
    has_accel = all(col in df.columns for col in ['accel_x', 'accel_y', 'accel_z'])
    has_quat = all(col in df.columns for col in ['quat_x', 'quat_y', 'quat_z', 'quat_w'])
    has_euler = all(col in df.columns for col in ['roll', 'pitch', 'yaw'])
    has_gravity = all(col in df.columns for col in ['grav_x', 'grav_y', 'grav_z'])
    has_lin_accel = all(col in df.columns for col in ['lin_accel_x', 'lin_accel_y', 'lin_accel_z'])
    has_mag = all(col in df.columns for col in ['mag_x', 'mag_y', 'mag_z'])
    has_proj_grav = all(col in df.columns for col in ['proj_grav_x', 'proj_grav_y', 'proj_grav_z'])
    
    # If fields_to_plot is None, enable all available fields except linear acceleration
    if fields_to_plot is None:
        fields_to_plot = {
            'gyro': has_gyro,
            'accel': has_accel,
            'quat': has_quat,
            'euler': has_euler,
            'gravity': has_gravity,
            'lin_accel': False,  # Disable linear acceleration by default
            'mag': has_mag,
            'proj_grav': has_proj_grav
        }
    
    # Calculate accelerometer frequency
    accel_freq_df = calculate_accel_frequency(df)
    
    # Calculate average accelerometer frequency
    avg_accel_freq = None
    if accel_freq_df is not None and not accel_freq_df.empty:
        avg_accel_freq = accel_freq_df['freq'].mean()
    
    # Plot 1: Raw data (gyro, accel, magnetometer)
    plot_raw_data(df, output_dir, fields_to_plot, has_gyro, has_accel, has_mag, avg_accel_freq, title)
    
    # Plot 2: Projected gravity data
    plot_gravity_data(df, output_dir, fields_to_plot, has_proj_grav, title)
    
    # Plot 3: Quaternion and Euler angles
    plot_orientation_angles(df, output_dir, fields_to_plot, has_quat, has_euler, title)
    
    print(f"Plots saved to {output_dir} directory")

def plot_raw_data(df, output_dir, fields_to_plot, has_gyro, has_accel, has_mag, avg_accel_freq=None, title=None):
    """Plot gyroscope, accelerometer, and magnetometer data in a single figure"""
    # Count how many subplots we need
    n_plots = sum([
        1 if has_gyro and fields_to_plot.get('gyro', False) else 0,
        1 if has_accel and fields_to_plot.get('accel', False) else 0,
        1 if has_mag and fields_to_plot.get('mag', False) else 0
    ])
    
    if n_plots > 0:
        fig, axes = plt.subplots(n_plots, 1, figsize=(12, 4*n_plots), sharex=True)
        
        # If only one subplot, make axes subscriptable
        if n_plots == 1:
            axes = [axes]
            
        plot_idx = 0
        
        # Plot gyroscope data
        if has_gyro and fields_to_plot.get('gyro', False):
            axes[plot_idx].plot(df['time'], df['gyro_x'], label='X-axis')
            axes[plot_idx].plot(df['time'], df['gyro_y'], label='Y-axis')
            axes[plot_idx].plot(df['time'], df['gyro_z'], label='Z-axis')
            axes[plot_idx].set_ylabel('Angular Velocity')
            axes[plot_idx].set_title('BNO055 - Gyroscope Data')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
        
        # Plot accelerometer data
        if has_accel and fields_to_plot.get('accel', False):
            axes[plot_idx].plot(df['time'], df['accel_x'], label='X-axis')
            axes[plot_idx].plot(df['time'], df['accel_y'], label='Y-axis')
            axes[plot_idx].plot(df['time'], df['accel_z'], label='Z-axis')
            axes[plot_idx].set_ylabel('Acceleration')
            
            # Add average frequency to the title if available
            title = 'BNO055 - Accelerometer Data'
            if avg_accel_freq is not None:
                title += f' (Avg. Freq: {avg_accel_freq:.2f} Hz)'
            axes[plot_idx].set_title(title)
            
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
            
        # Plot magnetometer data
        if has_mag and fields_to_plot.get('mag', False):
            axes[plot_idx].plot(df['time'], df['mag_x'], label='X-axis')
            axes[plot_idx].plot(df['time'], df['mag_y'], label='Y-axis')
            axes[plot_idx].plot(df['time'], df['mag_z'], label='Z-axis')
            axes[plot_idx].set_ylabel('Magnetic Field')
            axes[plot_idx].set_title('BNO055 - Magnetometer Data')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
        
        # Add x label to bottom subplot
        axes[-1].set_xlabel('Time')
            
        # Apply custom title if provided
        if title:
            fig.suptitle(title, fontsize=16)
            plt.subplots_adjust(top=0.95)  # Make room for suptitle
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'raw_data.png'), dpi=300)
        plt.close()

def plot_gravity_data(df, output_dir, fields_to_plot, has_proj_grav, title=None):
    """Plot projected gravity data and raw gravity data in separate subplots"""
    has_gravity = all(col in df.columns for col in ['grav_x', 'grav_y', 'grav_z'])
    
    # Check if we have data to plot
    if not has_proj_grav and not has_gravity:
        return
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    
    # Plot projected gravity vectors
    if has_proj_grav and fields_to_plot.get('proj_grav', False):
        axes[0].plot(df['time'], df['proj_grav_x'], label='X-axis')
        axes[0].plot(df['time'], df['proj_grav_y'], label='Y-axis')
        axes[0].plot(df['time'], df['proj_grav_z'], label='Z-axis')
        axes[0].set_ylabel('Projected Gravity Vector')
        axes[0].set_title('BNO055 - Calculated Projected Gravity')
        axes[0].legend()
        axes[0].grid(True)
    
    # Plot raw gravity vectors
    if has_gravity and fields_to_plot.get('gravity', False):
        axes[1].plot(df['time'], df['grav_x'], label='X-axis')
        axes[1].plot(df['time'], df['grav_y'], label='Y-axis')
        axes[1].plot(df['time'], df['grav_z'], label='Z-axis')
        axes[1].set_ylabel('Gravity Vector')
        axes[1].set_title('BNO055 - Sensor Projected Gravity')
        axes[1].legend()
        axes[1].grid(True)
    
    # Add x label to bottom subplot
    axes[1].set_xlabel('Time')
        
    # Apply custom title if provided
    if title:
        fig.suptitle(title, fontsize=16)
        plt.subplots_adjust(top=0.95)  # Make room for suptitle
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'gravity_data.png'), dpi=300)
    plt.close()

def plot_orientation_angles(df, output_dir, fields_to_plot, has_quat, has_euler, title=None):
    """Plot quaternion and Euler angles in separate subplots for roll, pitch, and yaw"""
    if not has_quat and not has_euler:
        return
    
    # Only continue if we have euler angles or quaternions to calculate them
    has_either = has_euler or (has_quat and fields_to_plot.get('quat', False))
    if not has_either:
        return
    
    # Calculate Euler angles from quaternions if available
    eq_roll, eq_pitch, eq_yaw = [], [], []
    
    if has_quat and fields_to_plot.get('quat', False):
        for idx, row in df.iterrows():
            # Skip if quaternion data is missing/zero
            if (np.isnan([row['quat_w'], row['quat_x'], row['quat_y'], row['quat_z']]).any() or 
                (row['quat_w'] == 0 and row['quat_x'] == 0 and row['quat_y'] == 0 and row['quat_z'] == 0)):
                eq_roll.append(np.nan)
                eq_pitch.append(np.nan)
                eq_yaw.append(np.nan)
                continue
            
            # Create quaternion and convert to Euler angles
            q = np.array([row['quat_w'], row['quat_x'], row['quat_y'], row['quat_z']])
            rot = Rotation.from_quat([q[1], q[2], q[3], q[0]], scalar_first=False)  # scipy uses [x,y,z,w] format
            angles = rot.as_euler('ZXY', degrees=True)
            
            eq_roll.append(angles[1])
            eq_pitch.append(angles[2])
            eq_yaw.append(angles[0])
    
    # Create figure with 4 subplots: 3 for Euler angles and 1 for quaternions
    fig, axs = plt.subplots(4, 1, figsize=(12, 16), sharex=True, gridspec_kw={'height_ratios': [1, 1, 1, 0.8]})
    
    # Apply custom title if provided
    title_text = title if title else 'BNO055 - Orientation Data Comparison'
    fig.suptitle(title_text, fontsize=16)
    
    # Plot Roll - both raw and calculated
    if has_euler and fields_to_plot.get('euler', False):
        axs[0].plot(df['time'], df['roll'], label='Raw Sensor', color='blue')
    if has_quat and fields_to_plot.get('quat', False):
        axs[0].plot(df['time'], eq_roll, label='From Quaternion', color='red', linestyle='--')
    axs[0].set_ylabel('Angle (degrees)')
    axs[0].set_title('Roll')
    axs[0].legend()
    axs[0].grid(True)
    
    # Plot Pitch - both raw and calculated
    if has_euler and fields_to_plot.get('euler', False):
        axs[1].plot(df['time'], df['pitch'], label='Raw Sensor', color='blue')
    if has_quat and fields_to_plot.get('quat', False):
        axs[1].plot(df['time'], eq_pitch, label='From Quaternion', color='red', linestyle='--')
    axs[1].set_ylabel('Angle (degrees)')
    axs[1].set_title('Pitch')
    axs[1].legend()
    axs[1].grid(True)
    
    # Plot Yaw - both raw and calculated
    if has_euler and fields_to_plot.get('euler', False):
        axs[2].plot(df['time'], df['yaw'], label='Raw Sensor', color='blue')
    if has_quat and fields_to_plot.get('quat', False):
        axs[2].plot(df['time'], eq_yaw, label='From Quaternion', color='red', linestyle='--')
    axs[2].set_ylabel('Angle (degrees)')
    axs[2].set_title('Yaw')
    axs[2].legend()
    axs[2].grid(True)
    
    # Plot quaternion data
    if has_quat and fields_to_plot.get('quat', False):
        axs[3].plot(df['time'], df['quat_x'], label='X')
        axs[3].plot(df['time'], df['quat_y'], label='Y')
        axs[3].plot(df['time'], df['quat_z'], label='Z')
        axs[3].plot(df['time'], df['quat_w'], label='W')
        axs[3].set_ylabel('Quaternion')
        axs[3].set_title('Sensor Quaternion')
        axs[3].legend()
        axs[3].grid(True)
    
    # Add x label to bottom subplot
    axs[3].set_xlabel('Time')
    
    plt.subplots_adjust(top=0.95)  # Make room for suptitle
    plt.savefig(os.path.join(output_dir, 'orientation_angles.png'), dpi=300)
    plt.close()
    
    # No longer need to create a separate quaternion plot

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Plot IMU data from CSV file')
    parser.add_argument('--file', type=str, default='imu_data.csv', help='Path to the CSV file containing IMU data')
    parser.add_argument('--output', type=str, default='plots', help='Directory to save the output PNG files (if --title is set, this is ignored and title_plots is used)')
    parser.add_argument('--title', type=str, default=None, help='Title to be applied to all plots and used to name the output folder (title_plots)')
    
    # Parse arguments
    args = parser.parse_args()
    
    # Example usage with all fields enabled
    plot_imu_data(csv_file=args.file, output_dir=args.output, title=args.title)
    
    # Example usage with only specific fields
    # plot_imu_data('imu.csv', fields_to_plot={
    #     'gyro': True, 
    #     'accel': True, 
    #     'quat': True, 
    #     'euler': True, 
    #     'gravity': True,
    #     'lin_accel': False,  # Linear acceleration disabled
    #     'mag': True,
    #     'proj_grav': True
    # })
