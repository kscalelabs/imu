use hexmove::{HexmoveImuReader, ImuReader, Quaternion, Vector3};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), String> {
    // Create a new HexmoveImuReader for the 'can0' interface
    let imu_reader = HexmoveImuReader::new("can0", 1, 1)
        .map_err(|e| format!("Failed to initialize IMU reader: {}", e))?;

    // Continuously read and print IMU data
    loop {
        let data = imu_reader
            .get_data()
            .map_err(|e| format!("Failed to get IMU data: {}", e))?;
        let angles = data.euler.unwrap_or(Vector3::default());
        let velocities = data.gyroscope.unwrap_or(Vector3::default());
        let accelerations = data.accelerometer.unwrap_or(Vector3::default());
        let quaternion = data.quaternion.unwrap_or(Quaternion::default());
        println!(
            "Angular Position: X={}°, Y={}°, Z={}° | Angular Velocity: X={}°/s, Y={}°/s, Z={}°/s | Acceleration: X={}m/s², Y={}m/s², Z={}m/s² | Quaternion: W={}, X={}, Y={}, Z={}",
            angles.x,
            angles.y,
            angles.z,
            velocities.x,
            velocities.y,
            velocities.z,
            accelerations.x,
            accelerations.y,
            accelerations.z,
            quaternion.w,
            quaternion.x,
            quaternion.y,
            quaternion.z
        );

        // Sleep for a short duration to avoid spamming the console
        thread::sleep(Duration::from_millis(500));
    }
}
