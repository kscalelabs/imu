#!/usr/bin/env python3
"""
Test script for LSM6DSV IMU driver
"""

import sys
import os
import time

# Add the project root to the path so we can import the imu module
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

try:
    from imu import create_lsm6dsvtr, Vector3, Quaternion
    print("✓ Successfully imported LSM6DSV functions")
except ImportError as e:
    print(f"✗ Failed to import LSM6DSV functions: {e}")
    sys.exit(1)

def test_lsm6dsv_creation():
    """Test creating an LSM6DSV reader instance"""
    try:
        # Try to create an LSM6DSV reader (this will fail if SPI device doesn't exist)
        reader = create_lsm6dsvtr("/dev/spidev1.0")
        print("✓ Successfully created LSM6DSV reader")
        return reader
    except Exception as e:
        print(f"✗ Failed to create LSM6DSV reader: {e}")
        print("  This is expected if the SPI device doesn't exist or permissions are insufficient")
        return None

def test_data_reading(reader):
    """Test reading data from the LSM6DSV"""
    if reader is None:
        print("⚠ Skipping data reading test (no reader available)")
        return
    
    try:
        # Try to read data
        data = reader.get_data()
        print("✓ Successfully read data from LSM6DSV")
        
        # Print available data
        if data.get('accelerometer'):
            accel = data['accelerometer']
            print(f"  Accelerometer: x={accel.x:.3f}, y={accel.y:.3f}, z={accel.z:.3f} m/s²")
        
        if data.get('gyroscope'):
            gyro = data['gyroscope']
            print(f"  Gyroscope: x={gyro.x:.3f}, y={gyro.y:.3f}, z={gyro.z:.3f} deg/s")
        
        if data.get('temperature') is not None:
            temp = data['temperature']
            print(f"  Temperature: {temp:.1f}°C")
            
    except Exception as e:
        print(f"✗ Failed to read data from LSM6DSV: {e}")

def main():
    print("LSM6DSV IMU Driver Test")
    print("=" * 30)
    
    # Test 1: Import functions
    print("\n1. Testing imports...")
    print("✓ All imports successful")
    
    # Test 2: Create reader
    print("\n2. Testing reader creation...")
    reader = test_lsm6dsv_creation()
    
    # Test 3: Read data
    print("\n3. Testing data reading...")
    test_data_reading(reader)
    
    # Test 4: Cleanup
    if reader:
        print("\n4. Testing cleanup...")
        try:
            reader.stop()
            print("✓ Successfully stopped reader")
        except Exception as e:
            print(f"✗ Failed to stop reader: {e}")
    
    print("\n" + "=" * 30)
    print("Test completed!")

if __name__ == "__main__":
    main()
