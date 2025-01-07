use linux_bno055::Bno055;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a new BNO055 instance
    let mut imu = Bno055::new("/dev/i2c-1")?;

    println!("Reading IMU data...");
    println!("Press Ctrl+C to exit");

    loop {
        // Read quaternion data
        if let Ok(quat) = imu.get_quaternion() {
            println!(
                "Quaternion: w: {:.3}, x: {:.3}, y: {:.3}, z: {:.3}",
                quat.w, quat.x, quat.y, quat.z
            );
        }

        // Read euler angles
        if let Ok(euler) = imu.get_euler_angles() {
            println!(
                "Euler Angles: roll: {:.3}°, pitch: {:.3}°, yaw: {:.3}°",
                euler.roll, euler.pitch, euler.yaw
            );
        }

        // Read raw sensor data
        if let Ok(accel) = imu.get_accelerometer() {
            println!(
                "Accelerometer: x: {:.3} m/s², y: {:.3} m/s², z: {:.3} m/s²",
                accel.x, accel.y, accel.z
            );
        }

        if let Ok(gyro) = imu.get_gyroscope() {
            println!(
                "Gyroscope: x: {:.3} °/s, y: {:.3} °/s, z: {:.3} °/s",
                gyro.x, gyro.y, gyro.z
            );
        }

        if let Ok(mag) = imu.get_magnetometer() {
            println!(
                "Magnetometer: x: {:.3} µT, y: {:.3} µT, z: {:.3} µT",
                mag.x, mag.y, mag.z
            );
        }

        // Read linear acceleration
        if let Ok(accel) = imu.get_linear_acceleration() {
            println!(
                "Linear Acceleration: x: {:.3} m/s², y: {:.3} m/s², z: {:.3} m/s²",
                accel.x, accel.y, accel.z
            );
        }

        // Read temperature and calibration status
        if let Ok(temp) = imu.get_temperature() {
            println!("Temperature: {}°C", temp);
        }

        if let Ok(cal) = imu.get_calibration_status() {
            println!(
                "Calibration - Sys: {}, Gyro: {}, Accel: {}, Mag: {}",
                (cal >> 6) & 0x03,
                (cal >> 4) & 0x03,
                (cal >> 2) & 0x03,
                cal & 0x03
            );
        }

        println!("---");
        thread::sleep(Duration::from_millis(100));
    }
}
