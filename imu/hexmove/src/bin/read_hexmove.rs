use hexmove::ImuReader;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let imu_reader = HexmoveImuReader::new("can0", 1, 1)?;

    println!("Reading Hexmove IMU data via ImuReader trait...");

    loop {
        match imu_reader.get_data() {
             Ok(data) => {
                if let Some(euler) = data.euler {
                     println!(
                        "Angular Position: X={:.1}°, Y={:.1}°, Z={:.1}°",
                        euler.x, euler.y, euler.z
                    );
                } else {
                    println!("Angular Position: Not available");
                }
                if let Some(gyro) = data.gyroscope {
                    println!(
                        "Angular Velocity: X={:.2}°/s, Y={:.2}°/s, Z={:.2}°/s",
                        gyro.x, gyro.y, gyro.z
                    );
                } else {
                    println!("Angular Velocity: Not available");
                }
                 if let Some(accel) = data.accelerometer {
                    println!(
                        "Accelerometer: X={:.2} m/s², Y={:.2} m/s², Z={:.2} m/s²",
                        accel.x, accel.y, accel.z
                    );
                } else {
                    println!("Accelerometer: Not available");
                }
                 if let Some(quat) = data.quaternion {
                    println!(
                        "Quaternion: w={:.3}, x={:.3}, y={:.3}, z={:.3}",
                        quat.w, quat.x, quat.y, quat.z
                    );
                } else {
                    println!("Quaternion: Not available");
                }

                 println!("----------------------------------------");
            }
             Err(e) => {
                eprintln!("Error reading IMU data: {}", e);
            }
        }

        thread::sleep(Duration::from_millis(500));
    }
}
