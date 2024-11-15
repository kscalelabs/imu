use hexmove::ImuReader;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), String> {
    // Create a new ImuReader for the 'can0' interface
    let imu_reader = ImuReader::new("can0", 1, 1)
        .map_err(|e| format!("Failed to initialize IMU reader: {}", e))?;

    // Continuously read and print IMU data
    loop {
        let data = imu_reader
            .get_data()
            .map_err(|e| format!("Failed to get IMU data: {}", e))?;
        println!(
            "Angular Position: X={}°, Y={}°, Z={}° | Angular Velocity: X={}°/s, Y={}°/s, Z={}°/s",
            data.x_angle,
            data.y_angle,
            data.z_angle,
            data.x_velocity,
            data.y_velocity,
            data.z_velocity
        );

        // Sleep for a short duration to avoid spamming the console
        thread::sleep(Duration::from_millis(500));
    }
}
