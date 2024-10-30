import can
import matplotlib.pyplot as plt
from collections import deque
import time
import threading

def setup_can_interface(channel='can0', interface='socketcan'):
    """Set up the CAN interface."""
    try:
        bus = can.interface.Bus(channel=channel, interface=interface)
        print(f"Connected to CAN interface on {channel}")
        return bus
    except Exception as e:
        print(f"Failed to connect to CAN interface: {e}")
        return None

def data_receiver(bus, x_angle_data, y_angle_data, z_angle_data, x_velocity_data, y_velocity_data, z_velocity_data, timestamps_angle, timestamps_velocity):
    """Receive data from the IMU and store it in deques."""
    try:
        while True:
            message = bus.recv(timeout=1.0)  # Wait for a message
            if message is not None:
                current_time = time.time()

                if message.arbitration_id == 0x0B0101B1:
                    x_angle = int.from_bytes(message.data[0:2], byteorder='little', signed=True) * 0.01
                    y_angle = int.from_bytes(message.data[2:4], byteorder='little', signed=True) * 0.01
                    z_angle = int.from_bytes(message.data[4:6], byteorder='little', signed=True) * 0.01
                    x_angle_data.append(x_angle)
                    y_angle_data.append(y_angle)
                    z_angle_data.append(z_angle)
                    timestamps_angle.append(current_time)

                if message.arbitration_id == 0x0B0101B2:
                    x_velocity = int.from_bytes(message.data[0:2], byteorder='little', signed=True) * 0.01
                    y_velocity = int.from_bytes(message.data[2:4], byteorder='little', signed=True) * 0.01
                    z_velocity = int.from_bytes(message.data[4:6], byteorder='little', signed=True) * 0.01
                    x_velocity_data.append(x_velocity)
                    y_velocity_data.append(y_velocity)
                    z_velocity_data.append(z_velocity)
                    timestamps_velocity.append(current_time)

    except Exception as e:
        print(f"Error receiving IMU data: {e}")

def plot_data(x_angle_data, y_angle_data, z_angle_data, x_velocity_data, y_velocity_data, z_velocity_data, timestamps_angle, timestamps_velocity):
    """Plot the angular position and velocity."""
    plt.ion()  # Turn on interactive mode
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    try:
        while True:
            if len(timestamps_angle) > 0 and len(timestamps_velocity) > 0:
                ax1.clear()
                ax2.clear()

                flat_x_angle = [x for x in x_angle_data]
                flat_y_angle = [y for y in y_angle_data]
                flat_z_angle = [z for z in z_angle_data]

                flat_timestamps_angle = [x for x in timestamps_angle]

                flat_timestamps_velocity = [x for x in timestamps_velocity]

                ax1.plot(flat_timestamps_angle, flat_x_angle, label='X Angle')
                ax1.plot(flat_timestamps_angle, flat_y_angle, label='Y Angle')
                ax1.plot(flat_timestamps_angle, flat_z_angle, label='Z Angle')
                ax1.set_title('Angular Position')
                ax1.set_ylabel('Degrees')
                ax1.legend()

                flat_x_velocity = [x for x in x_velocity_data]
                flat_y_velocity = [y for y in y_velocity_data]
                flat_z_velocity = [z for z in z_velocity_data]

                ax2.plot(flat_timestamps_velocity, flat_x_velocity, label='X Velocity')
                ax2.plot(flat_timestamps_velocity, flat_y_velocity, label='Y Velocity')
                ax2.plot(flat_timestamps_velocity, flat_z_velocity, label='Z Velocity')
                ax2.set_title('Angular Velocity')
                ax2.set_ylabel('Degrees/s')
                ax2.set_xlabel('Time (s)')
                ax2.legend()

                plt.pause(0.1)  # Pause to update the plot

    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        plt.ioff()
        plt.show()

def main():
    bus = setup_can_interface()
    if bus:
        time_window = 10
        x_angle_data = deque(maxlen=time_window * 10)
        y_angle_data = deque(maxlen=time_window * 10)
        z_angle_data = deque(maxlen=time_window * 10)
        x_velocity_data = deque(maxlen=time_window * 10)
        y_velocity_data = deque(maxlen=time_window * 10)
        z_velocity_data = deque(maxlen=time_window * 10)
        timestamps_angle = deque(maxlen=time_window * 10)
        timestamps_velocity = deque(maxlen=time_window * 10)

        # Start the data receiving thread
        data_thread = threading.Thread(target=data_receiver, args=(bus, x_angle_data, y_angle_data, z_angle_data, x_velocity_data, y_velocity_data, z_velocity_data, timestamps_angle, timestamps_velocity))
        data_thread.daemon = True
        data_thread.start()

        # Start the plotting function
        plot_data(x_angle_data, y_angle_data, z_angle_data, x_velocity_data, y_velocity_data, z_velocity_data, timestamps_angle, timestamps_velocity)

if __name__ == "__main__":
    main()
