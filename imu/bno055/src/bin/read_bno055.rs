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

        // Read linear acceleration
        if let Ok(accel) = imu.get_linear_acceleration() {
            println!(
                "Linear Acceleration: x: {:.3} m/s², y: {:.3} m/s², z: {:.3} m/s²",
                accel.x, accel.y, accel.z
            );
        }

        println!("---");
        thread::sleep(Duration::from_millis(100));
    }
}
