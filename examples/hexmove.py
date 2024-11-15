"""Example usage of the Hexmove IMU."""

import time

from imu import HexmoveImuReader


def main() -> None:
    # Initialize the IMU reader for the 'can0' interface and imu with serial number 1 and model 1
    try:
        imu_reader = HexmoveImuReader("can0", 1, 1)
    except Exception as e:
        print(f"Failed to initialize IMU reader: {e}")
        return

    imu_reader.zero_imu(duration_ms=1000)

    try:
        while True:
            # # Get the current IMU data
            # data = imu_reader.get_data()

            # angles = [data.x_angle, data.y_angle, data.z_angle]
            # velocities = [data.x_velocity, data.y_velocity, data.z_velocity]

            angles = imu_reader.get_angles()
            velocities = imu_reader.get_velocities()
            print(
                f"Angular Position: X={angles[0]}°, Y={angles[1]}°, Z={angles[2]}° | "
                f"Angular Velocity: X={velocities[0]}°/s, Y={velocities[1]}°/s, Z={velocities[2]}°/s"
            )

            # Sleep for a short duration to avoid spamming the console
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping IMU reader...")
    finally:
        imu_reader.stop()


if __name__ == "__main__":
    main()
