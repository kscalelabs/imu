use linux_bmi088::Bmi088Reader;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let imu = Bmi088Reader::new("/dev/i2c-1")?;
    println!("Reading BMI088 sensor data via ImuReader trait...");

    loop {
        match imu.get_data() {
            Ok(data) => {
                if let Some(quat) = data.quaternion {
                    println!(
                        "Quaternion: w={:.3}, x={:.3}, y={:.3}, z={:.3}",
                        quat.w, quat.x, quat.y, quat.z
                    );
                } else {
                    println!("Quaternion: Not available");
                }

                if let Some(accel) = data.accelerometer {
                    println!(
                        "Accel: x={:.2} g, y={:.2} g, z={:.2} g",
                        accel.x, accel.y, accel.z
                    );
                } else {
                    println!("Accel: Not available");
                }

                if let Some(gyro) = data.gyroscope {
                    println!(
                        "Gyro:  x={:.2} °/s, y={:.2} °/s, z={:.2} °/s",
                        gyro.x, gyro.y, gyro.z
                    );
                } else {
                    println!("Gyro: Not available");
                }

                if let Some(euler) = data.euler {
                    println!(
                        "Euler: roll={:.1}°, pitch={:.1}°, yaw={:.1}°",
                        euler.x, euler.y, euler.z
                    );
                } else {
                    println!("Euler: Not available");
                }

                if let Some(temp) = data.temperature {
                    println!("Temp:  {:.1} °C", temp);
                } else {
                    println!("Temp: Not available");
                }

                println!("----------------------------------------");
            }
            Err(e) => {
                eprintln!("Error reading IMU data: {}", e);
            }
        }

        thread::sleep(Duration::from_millis(100));
    }
}
