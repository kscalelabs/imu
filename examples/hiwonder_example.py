"""Example usage of the Hiwonder IMU."""

import time

import imu


def main() -> None:
    # Create IMU instance
    reader = imu.create_hiwonder("/dev/ttyUSB0", 9600)

    try:
        while True:
            if data := reader.get_data():
                print("\033[2J\033[H")  # Clear screen
                print(f"Acceleration (m/s²): {data.accelerometer}")
                print(f"Gyroscope (deg/s):  {data.gyroscope}")
                print(f"Angle (degrees):     {data.angle}")
                print(f"Quaternion: {data.quaternion}")
                time.sleep(0.1)  # Add small delay to make output readable
    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    main()
