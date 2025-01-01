from bindings_hiwonder import HiwonderImu 
import time

def main():
    # Create IMU instance
    imu = HiwonderImu("/dev/ttyUSB0", 9600)
    
    try:
        while True:
            if data := imu.read_data():
                acc, gyro, angle = data
                print(f"\033[2J\033[H")  # Clear screen
                print(f"Acceleration (m/s²): {acc}")
                print(f"Gyroscope (deg/s):  {gyro}")
                print(f"Angle (degrees):     {angle}")
                time.sleep(0.1)  # Add small delay to make output readable
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()