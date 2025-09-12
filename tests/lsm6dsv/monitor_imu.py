#!/usr/bin/env python3
"""
Real-time IMU monitoring script
Continuously reads and displays the current state of the LSM6DSV IMU
"""

import sys
import os
import time
import signal

# Add the project root to the path so we can import the imu module
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

try:
    from imu import create_lsm6dsvtr, Vector3, Quaternion
    print("✓ Successfully imported LSM6DSV functions")
except ImportError as e:
    print(f"✗ Failed to import LSM6DSV functions: {e}")
    sys.exit(1)

class IMUMonitor:
    def __init__(self, spi_device="/dev/spidev1.0", update_rate=0.1):
        """Initialize the IMU monitor
        
        Args:
            spi_device: SPI device path (default: /dev/spidev1.0)
            update_rate: Update rate in seconds (default: 0.1 = 10 Hz)
        """
        self.spi_device = spi_device
        self.update_rate = update_rate
        self.reader = None
        self.running = False
        
    def connect(self):
        """Connect to the IMU"""
        try:
            print(f"Connecting to LSM6DSV on {self.spi_device}...")
            self.reader = create_lsm6dsvtr(self.spi_device)
            print("✓ Successfully connected to LSM6DSV")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to LSM6DSV: {e}")
            return False
    
    def format_vector(self, vector, unit=""):
        """Format a Vector3 for display"""
        if vector is None:
            return "N/A"
        return f"x={vector.x:8.3f}, y={vector.y:8.3f}, z={vector.z:8.3f} {unit}"
    
    def format_quaternion(self, quat):
        """Format a Quaternion for display"""
        if quat is None:
            return "N/A"
        return f"w={quat.w:6.3f}, x={quat.x:6.3f}, y={quat.y:6.3f}, z={quat.z:6.3f}"
    
    def display_data(self, data):
        """Display the IMU data in a formatted way"""
        # Clear screen and move cursor to top
        print("\033[2J\033[H", end="")
        
        print("=" * 80)
        print("                    LSM6DSV IMU Real-Time Monitor")
        print("=" * 80)
        print(f"Device: {self.spi_device} | Update Rate: {1/self.update_rate:.1f} Hz")
        print("=" * 80)
        
        # Accelerometer data
        if data.get('accelerometer'):
            accel = data['accelerometer']
            print(f"Accelerometer (m/s²): {self.format_vector(accel, 'm/s²')}")
        else:
            print("Accelerometer: N/A")
        
        # Gyroscope data
        if data.get('gyroscope'):
            gyro = data['gyroscope']
            print(f"Gyroscope (deg/s):    {self.format_vector(gyro, 'deg/s')}")
        else:
            print("Gyroscope: N/A")
        
        # Magnetometer data
        if data.get('magnetometer'):
            mag = data['magnetometer']
            print(f"Magnetometer (μT):    {self.format_vector(mag, 'μT')}")
        else:
            print("Magnetometer: N/A")
        
        # Quaternion data
        if data.get('quaternion'):
            quat = data['quaternion']
            print(f"Quaternion:           {self.format_quaternion(quat)}")
        else:
            print("Quaternion: N/A")
        
        # Euler angles
        if data.get('euler'):
            euler = data['euler']
            print(f"Euler Angles (deg):   {self.format_vector(euler, 'deg')}")
        else:
            print("Euler Angles: N/A")
        
        # Linear acceleration
        if data.get('linear_acceleration'):
            lin_accel = data['linear_acceleration']
            print(f"Linear Accel (m/s²):  {self.format_vector(lin_accel, 'm/s²')}")
        else:
            print("Linear Acceleration: N/A")
        
        # Gravity vector
        if data.get('gravity'):
            gravity = data['gravity']
            print(f"Gravity (m/s²):       {self.format_vector(gravity, 'm/s²')}")
        else:
            print("Gravity: N/A")
        
        # Temperature
        if data.get('temperature') is not None:
            temp = data['temperature']
            print(f"Temperature:          {temp:6.1f}°C")
        else:
            print("Temperature: N/A")
        
        # Calibration status
        if data.get('calibration_status') is not None:
            cal = data['calibration_status']
            print(f"Calibration Status:   {cal}")
        else:
            print("Calibration Status: N/A")
        
        print("=" * 80)
        print("Press Ctrl+C to stop monitoring")
        print("=" * 80)
    
    def run(self):
        """Run the continuous monitoring loop"""
        if not self.connect():
            return
        
        self.running = True
        
        # Set up signal handler for graceful shutdown
        def signal_handler(sig, frame):
            print("\n\nShutting down gracefully...")
            self.running = False
        
        signal.signal(signal.SIGINT, signal_handler)
        
        try:
            while self.running:
                try:
                    # Read data from IMU
                    data = self.reader.get_data()
                    self.display_data(data)
                    
                    # Wait for next update
                    time.sleep(self.update_rate)
                    
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"\nError reading IMU data: {e}")
                    print("Retrying in 1 second...")
                    time.sleep(1)
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.reader:
            try:
                self.reader.stop()
                print("✓ IMU reader stopped successfully")
            except Exception as e:
                print(f"Warning: Error stopping IMU reader: {e}")

def main():
    """Main function"""
    print("LSM6DSV IMU Real-Time Monitor")
    print("=" * 40)
    
    # Parse command line arguments
    spi_device = "/dev/spidev1.0"
    update_rate = 0.1  # 10 Hz
    
    if len(sys.argv) > 1:
        spi_device = sys.argv[1]
    
    if len(sys.argv) > 2:
        try:
            update_rate = float(sys.argv[2])
        except ValueError:
            print("Error: Update rate must be a number")
            sys.exit(1)
    
    print(f"Using SPI device: {spi_device}")
    print(f"Update rate: {1/update_rate:.1f} Hz")
    print()
    
    # Create and run monitor
    monitor = IMUMonitor(spi_device, update_rate)
    monitor.run()

if __name__ == "__main__":
    main()
