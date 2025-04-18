import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse
from scipy.spatial.transform import Rotation

def calculate_accel_frequency(df, accel_prefix='accel'):
    """
    Calculate the frequency of accelerometer data by measuring time differences
    between consecutive unique data points.
    
    Parameters:
    -----------
    df : pandas.DataFrame
        DataFrame containing accelerometer data with timestamp column
    accel_prefix : str
        Prefix for accelerometer columns (e.g., 'accel', 'hw_acc', 'bno_acc')
    
    Returns:
    --------
    pandas.DataFrame
        DataFrame with time and frequency columns
    """
    # Check column names based on prefix
    accel_cols = [col for col in df.columns if col.startswith(f"{accel_prefix}_") and col.endswith(('x', 'y', 'z'))]
    
    # Check if accelerometer data exists
    if len(accel_cols) != 3:
        return None
    
    # Create a copy with only timestamp and accelerometer data
    accel_df = df[['timestamp', 'time'] + accel_cols].copy()
    
    # Create a combined value to detect changes in any axis
    accel_df['accel_combined'] = accel_df[accel_cols[0]].astype(str) + '_' + \
                                 accel_df[accel_cols[1]].astype(str) + '_' + \
                                 accel_df[accel_cols[2]].astype(str)
    
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

def plot_imu_data(csv_file='imu_data.csv', output_dir='plots', fields_to_plot=None, title=None, imu_config=None):
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
    imu_config : dict
        Configuration for multiple IMUs
        Example: {
            'hw': {'prefix': 'hw_', 'label': 'Hardware IMU', 'color': 'blue'},
            'bno': {'prefix': 'bno_', 'label': 'BNO055', 'color': 'red'}
        }
        If None, will auto-detect IMUs based on column names
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
    
    # Auto-detect IMUs if no config provided
    if imu_config is None:
        imu_config = {}
        
        # Check for BNO055 IMU
        if any(col.startswith('bno_') for col in df.columns):
            imu_config['bno'] = {'prefix': 'bno_', 'label': 'BNO055', 'color': 'red'}
            
        # Check for hardware IMU
        if any(col.startswith('hw_') for col in df.columns):
            imu_config['hw'] = {'prefix': 'hw_', 'label': 'Hiwonder IMU', 'color': 'blue'}
            
        # Check for generic IMU data (original format)
        if any(col in df.columns for col in ['accel_x', 'gyro_x']):
            imu_config['generic'] = {'prefix': '', 'label': 'IMU', 'color': 'green'}
    
    # If no IMUs detected, return
    if not imu_config:
        print("No IMU data detected in the CSV file.")
        return
    
    # For each detected IMU, check available fields
    for imu_id, config in imu_config.items():
        prefix = config['prefix']
        
        # Map different possible column naming patterns
        accel_pattern = next((p for p in [f"{prefix}acc_", f"{prefix}accel_"] if any(col.startswith(p) for col in df.columns)), None)
        gyro_pattern = next((p for p in [f"{prefix}gyro_"] if any(col.startswith(p) for col in df.columns)), None)
        quat_pattern = next((p for p in [f"{prefix}quat_"] if any(col.startswith(p) for col in df.columns)), None)
        
        # Store the patterns in the config
        config['patterns'] = {
            'accel': accel_pattern,
            'gyro': gyro_pattern,
            'quat': quat_pattern
        }
        
        # Check available fields for this IMU
        config['has_gyro'] = gyro_pattern is not None and all(f"{gyro_pattern}{axis}" in df.columns for axis in ['x', 'y', 'z'])
        config['has_accel'] = accel_pattern is not None and all(f"{accel_pattern}{axis}" in df.columns for axis in ['x', 'y', 'z'])
        config['has_quat'] = quat_pattern is not None and all(f"{quat_pattern}{comp}" in df.columns for comp in ['x', 'y', 'z', 'w'])
        
        # Check for Euler angles (different naming conventions)
        if all(f"{prefix}{angle}" in df.columns for angle in ['roll', 'pitch', 'yaw']):
            config['has_euler'] = True
            config['euler_pattern'] = [f"{prefix}roll", f"{prefix}pitch", f"{prefix}yaw"]
        elif all(f"{prefix}angle_{axis}" in df.columns for axis in ['x', 'y', 'z']):
            config['has_euler'] = True
            config['euler_pattern'] = [f"{prefix}angle_x", f"{prefix}angle_y", f"{prefix}angle_z"]
        else:
            config['has_euler'] = False
            config['euler_pattern'] = []
        
        # Check for gravity data
        gravity_pattern = next((p for p in [f"{prefix}grav_"] if any(col.startswith(p) for col in df.columns)), None)
        config['has_gravity'] = gravity_pattern is not None and all(f"{gravity_pattern}{axis}" in df.columns for axis in ['x', 'y', 'z'])
        config['gravity_pattern'] = gravity_pattern
        
        # Check for projected gravity data
        proj_gravity_pattern = next((p for p in [f"{prefix}proj_grav_"] if any(col.startswith(p) for col in df.columns)), None)
        config['has_proj_grav'] = proj_gravity_pattern is not None and all(f"{proj_gravity_pattern}{axis}" in df.columns for axis in ['x', 'y', 'z'])
        config['proj_grav_pattern'] = proj_gravity_pattern
        
        # Check for magnetometer data
        mag_pattern = next((p for p in [f"{prefix}mag_"] if any(col.startswith(p) for col in df.columns)), None)
        config['has_mag'] = mag_pattern is not None and all(f"{mag_pattern}{axis}" in df.columns for axis in ['x', 'y', 'z'])
        config['mag_pattern'] = mag_pattern
        
        # Check for linear acceleration data
        lin_accel_pattern = next((p for p in [f"{prefix}lin_acc_"] if any(col.startswith(p) for col in df.columns)), None)
        config['has_lin_accel'] = lin_accel_pattern is not None and all(f"{lin_accel_pattern}{axis}" in df.columns for axis in ['x', 'y', 'z'])
        config['lin_accel_pattern'] = lin_accel_pattern
    
    # If fields_to_plot is None, enable all available fields except linear acceleration
    if fields_to_plot is None:
        fields_to_plot = {
            'gyro': True,
            'accel': True,
            'quat': True,
            'euler': True,
            'gravity': True,
            'lin_accel': False,  # Disable linear acceleration by default
            'mag': True,
            'proj_grav': True
        }
    
    # Calculate accelerometer frequency for each IMU
    accel_freq_info = {}
    for imu_id, config in imu_config.items():
        if config['has_accel'] and config['patterns']['accel']:
            accel_freq_df = calculate_accel_frequency(df, accel_prefix=config['patterns']['accel'].rstrip('_'))
            if accel_freq_df is not None and not accel_freq_df.empty:
                accel_freq_info[imu_id] = accel_freq_df['freq'].mean()
    
    # Plot 1: Raw data (gyro, accel, magnetometer)
    plot_raw_data(df, output_dir, fields_to_plot, imu_config, accel_freq_info, title)
    
    # Plot 2: Gravity data
    plot_gravity_data(df, output_dir, fields_to_plot, imu_config, title)
    
    # Plot 3: Orientation angles
    plot_orientation_angles(df, output_dir, fields_to_plot, imu_config, title)
    
    print(f"Plots saved to {output_dir} directory")

def plot_raw_data(df, output_dir, fields_to_plot, imu_config, accel_freq_info=None, title=None):
    """Plot gyroscope, accelerometer, and magnetometer data with separate plots for each IMU"""
    # Create separate plots for each IMU, following original style
    for imu_id, config in imu_config.items():
        # Count how many subplots we need for this IMU
        n_plots = sum([
            1 if config.get('has_gyro', False) and fields_to_plot.get('gyro', False) else 0,
            1 if config.get('has_accel', False) and fields_to_plot.get('accel', False) else 0,
            1 if config.get('has_mag', False) and fields_to_plot.get('mag', False) else 0
        ])
        
        if n_plots == 0:
            continue  # Skip if no plots needed for this IMU
        
        fig, axes = plt.subplots(n_plots, 1, figsize=(12, 4*n_plots), sharex=True)
        
        # If only one subplot, make axes subscriptable
        if n_plots == 1:
            axes = [axes]
            
        plot_idx = 0
        
        # Plot gyroscope data
        if config.get('has_gyro', False) and fields_to_plot.get('gyro', False):
            gyro_pattern = config['patterns']['gyro']
            axes[plot_idx].plot(df['time'], df[f"{gyro_pattern}x"], label='X-axis', color='red')
            axes[plot_idx].plot(df['time'], df[f"{gyro_pattern}y"], label='Y-axis', color='green')
            axes[plot_idx].plot(df['time'], df[f"{gyro_pattern}z"], label='Z-axis', color='blue')
            axes[plot_idx].set_ylabel('Angular Velocity (rad/s)')
            axes[plot_idx].set_title(f'{config["label"]} - Gyroscope Data')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
        
        # Plot accelerometer data
        if config.get('has_accel', False) and fields_to_plot.get('accel', False):
            accel_pattern = config['patterns']['accel']
            axes[plot_idx].plot(df['time'], df[f"{accel_pattern}x"], label='X-axis', color='red')
            axes[plot_idx].plot(df['time'], df[f"{accel_pattern}y"], label='Y-axis', color='green')
            axes[plot_idx].plot(df['time'], df[f"{accel_pattern}z"], label='Z-axis', color='blue')
            axes[plot_idx].set_ylabel('Acceleration (m/s²)')
            
            # Add average frequency to the title if available
            title_text = f'{config["label"]} - Accelerometer Data'
            if accel_freq_info and imu_id in accel_freq_info:
                title_text += f' (Avg. Freq: {accel_freq_info[imu_id]:.2f} Hz)'
            axes[plot_idx].set_title(title_text)
            
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
            
        # Plot magnetometer data
        if config.get('has_mag', False) and fields_to_plot.get('mag', False):
            mag_pattern = config['mag_pattern']
            axes[plot_idx].plot(df['time'], df[f"{mag_pattern}x"], label='X-axis', color='red')
            axes[plot_idx].plot(df['time'], df[f"{mag_pattern}y"], label='Y-axis', color='green')
            axes[plot_idx].plot(df['time'], df[f"{mag_pattern}z"], label='Z-axis', color='blue')
            axes[plot_idx].set_ylabel('Magnetic Field (uT)')
            axes[plot_idx].set_title(f'{config["label"]} - Magnetometer Data')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
        
        # Add x label to bottom subplot
        axes[-1].set_xlabel('Time (s)')
            
        # Apply custom title if provided
        if title:
            fig.suptitle(f"{title} - {config['label']}", fontsize=16)
            plt.subplots_adjust(top=0.95)  # Make room for suptitle
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f'raw_data_{imu_id}.png'), dpi=300)
        plt.close()

def plot_gravity_data(df, output_dir, fields_to_plot, imu_config, title=None):
    """Plot projected gravity data and raw gravity data with separate plots for each IMU"""
    # Create separate plots for each IMU
    for imu_id, config in imu_config.items():
        # Check if this IMU has gravity data
        has_gravity = config.get('has_gravity', False)
        has_proj_grav = config.get('has_proj_grav', False)
        
        # Skip if no gravity data or not enabled in fields_to_plot
        if not ((has_gravity and fields_to_plot.get('gravity', False)) or 
                (has_proj_grav and fields_to_plot.get('proj_grav', False))):
            continue
        
        # Determine how many subplots we need
        n_plots = sum([
            1 if has_proj_grav and fields_to_plot.get('proj_grav', False) else 0,
            1 if has_gravity and fields_to_plot.get('gravity', False) else 0
        ])
        
        if n_plots == 0:
            continue  # Skip if no plots needed
            
        fig, axes = plt.subplots(n_plots, 1, figsize=(12, 4*n_plots), sharex=True)
        
        # If only one subplot, make axes subscriptable
        if n_plots == 1:
            axes = [axes]
            
        plot_idx = 0
            
        # Plot projected gravity vectors
        if has_proj_grav and fields_to_plot.get('proj_grav', False):
            proj_grav_pattern = config['proj_grav_pattern']
            axes[plot_idx].plot(df['time'], df[f"{proj_grav_pattern}x"], label='X-axis', color='red')
            axes[plot_idx].plot(df['time'], df[f"{proj_grav_pattern}y"], label='Y-axis', color='green')
            axes[plot_idx].plot(df['time'], df[f"{proj_grav_pattern}z"], label='Z-axis', color='blue')
            axes[plot_idx].set_ylabel('Projected Gravity Vector (g)')
            axes[plot_idx].set_title(f'{config["label"]} - Projected Gravity')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
        
        # Plot raw gravity vectors
        if has_gravity and fields_to_plot.get('gravity', False):
            gravity_pattern = config['gravity_pattern']
            axes[plot_idx].plot(df['time'], df[f"{gravity_pattern}x"], label='X-axis', color='red')
            axes[plot_idx].plot(df['time'], df[f"{gravity_pattern}y"], label='Y-axis', color='green')
            axes[plot_idx].plot(df['time'], df[f"{gravity_pattern}z"], label='Z-axis', color='blue')
            axes[plot_idx].set_ylabel('Gravity Vector (g)')
            axes[plot_idx].set_title(f'{config["label"]} - Raw Gravity')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
        
        # Add x label to bottom subplot
        axes[-1].set_xlabel('Time (s)')
            
        # Apply custom title if provided
        if title:
            fig.suptitle(f"{title} - {config['label']}", fontsize=16)
            plt.subplots_adjust(top=0.95)  # Make room for suptitle
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f'gravity_data_{imu_id}.png'), dpi=300)
        plt.close()

def plot_orientation_angles(df, output_dir, fields_to_plot, imu_config, title=None):
    """Plot quaternion and Euler angles with separate plots for each IMU"""
    # Create separate plots for each IMU
    for imu_id, config in imu_config.items():
        # Check if this IMU has orientation data
        has_quat = config.get('has_quat', False)
        has_euler = config.get('has_euler', False)
        
        # Skip if no orientation data or not enabled in fields_to_plot
        if not ((has_quat and fields_to_plot.get('quat', False)) or 
                (has_euler and fields_to_plot.get('euler', False))):
            continue
            
        # Calculate Euler angles from quaternions if needed
        eq_roll, eq_pitch, eq_yaw = None, None, None
        
        if has_quat and fields_to_plot.get('quat', False):
            # Get quaternion column names
            quat_pattern = config['patterns']['quat']
            quat_cols = [f"{quat_pattern}{comp}" for comp in ['w', 'x', 'y', 'z']]
            
            # Check if all columns exist
            if all(col in df.columns for col in quat_cols):
                # Initialize arrays for calculated Euler angles
                eq_roll, eq_pitch, eq_yaw = [], [], []
                
                # For Hiwonder IMU, use radians instead of degrees
                use_radians = (imu_id == 'hw')
                
                for idx, row in df.iterrows():
                    # Get quaternion values
                    qw = row[quat_cols[0]]
                    qx = row[quat_cols[1]]
                    qy = row[quat_cols[2]]
                    qz = row[quat_cols[3]]
                    
                    # Skip if quaternion data is missing/zero
                    if (np.isnan([qw, qx, qy, qz]).any() or 
                        (qw == 0 and qx == 0 and qy == 0 and qz == 0)):
                        eq_roll.append(np.nan)
                        eq_pitch.append(np.nan)
                        eq_yaw.append(np.nan)
                        continue
                    
                    # Create quaternion and convert to Euler angles
                    q = np.array([qw, qx, qy, qz])
                    if q[0] == 1 and q[1] == 0 and q[2] == 0 and q[3] == 0:
                        # Identity quaternion
                        eq_roll.append(0)
                        eq_pitch.append(0)
                        eq_yaw.append(0)
                    else:
                        rot = Rotation.from_quat([q[0], q[1], q[2], q[3]], scalar_first=True)  # scipy uses [x,y,z,w] format
                        
                        # Get angles in the appropriate unit
                        if use_radians:
                            angles = rot.as_euler('ZXY', degrees=False)  # Radians for Hiwonder
                            eq_roll.append(angles[1])
                            eq_pitch.append(angles[2])
                            eq_yaw.append(angles[0])
                        else:
                            angles = rot.as_euler('ZXY', degrees=True)  # Degrees for others
                            eq_roll.append(angles[1])
                            eq_pitch.append(angles[2])
                            eq_yaw.append(angles[0])
        
        # Create figure with 4 subplots: 3 for Euler angles and 1 for quaternions
        fig, axs = plt.subplots(4, 1, figsize=(12, 16), sharex=True, gridspec_kw={'height_ratios': [1, 1, 1, 0.8]})
        
        # Apply title
        title_text = f"{config['label']} - Orientation Data"
        if title:
            title_text = f"{title} - {config['label']}"
        fig.suptitle(title_text, fontsize=16)
        
        # Plot Euler angles (Roll, Pitch, Yaw)
        angle_names = ['Roll', 'Pitch', 'Yaw']
        euler_indices = [0, 1, 2]  # Mapping of roll, pitch, yaw to subplot indices
        
        # Set the y-axis label based on whether we're using radians or degrees for this IMU
        angle_unit = "radians" if imu_id == 'hw' else "degrees"
        
        # If we have euler angle columns, plot them
        if has_euler and fields_to_plot.get('euler', False):
            euler_pattern = config['euler_pattern']
            
            # Plot each angle (roll, pitch, yaw)
            for i, (angle_idx, angle_name) in enumerate(zip(euler_indices, angle_names)):
                if i < len(euler_pattern) and euler_pattern[i] in df.columns:
                    axs[angle_idx].plot(df['time'], df[euler_pattern[i]], 
                                        label='Raw Sensor', color='blue')
                                        
                # Also plot the calculated angle from quaternion if available
                if eq_roll is not None and i == 0:
                    axs[angle_idx].plot(df['time'], eq_roll, 
                                        label='From Quaternion', color='red', linestyle='--')
                elif eq_pitch is not None and i == 1:
                    axs[angle_idx].plot(df['time'], eq_pitch, 
                                        label='From Quaternion', color='red', linestyle='--')
                elif eq_yaw is not None and i == 2:
                    axs[angle_idx].plot(df['time'], eq_yaw, 
                                        label='From Quaternion', color='red', linestyle='--')
                
                axs[angle_idx].set_ylabel(f'Angle ({angle_unit})')
                axs[angle_idx].set_title(angle_name)
                axs[angle_idx].legend()
                axs[angle_idx].grid(True)
        
        # Plot quaternion data
        if has_quat and fields_to_plot.get('quat', False):
            quat_pattern = config['patterns']['quat']
            quat_components = ['x', 'y', 'z', 'w']
            quat_colors = ['red', 'green', 'blue', 'black']
            
            for comp, color in zip(quat_components, quat_colors):
                col_name = f"{quat_pattern}{comp}"
                if col_name in df.columns:
                    axs[3].plot(df['time'], df[col_name], 
                                label=f"{comp.upper()}", color=color)
            
            axs[3].set_ylabel('Quaternion')
            axs[3].set_title('Quaternion Components')
            axs[3].legend()
            axs[3].grid(True)
        
        # Add x label to bottom subplot
        axs[3].set_xlabel('Time (s)')
        
        plt.subplots_adjust(top=0.95, hspace=0.3)  # Make room for suptitle
        plt.savefig(os.path.join(output_dir, f'orientation_angles_{imu_id}.png'), dpi=300)
        plt.close()

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
