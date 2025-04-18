use linux_bno055::Bno055Reader;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let imu = Bno055Reader::new("/dev/i2c-1")?;

    println!("Reading BNO055 data via ImuReader trait...");
    println!("Press Ctrl+C to exit");

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

                if let Some(euler) = data.euler {
                    println!(
                        "Euler: roll={:.1}°, pitch={:.1}°, yaw={:.1}°",
                        euler.x, euler.y, euler.z // Assuming BaseVector3 maps roll->x, pitch->y, yaw->z
                    );
                } else {
                    println!("Euler: Not available");
                }

                if let Some(accel) = data.accelerometer {
                    println!(
                        "Accel: x={:.2} m/s², y={:.2} m/s², z={:.2} m/s²",
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

                 if let Some(lin_accel) = data.linear_acceleration {
                    println!(
                        "Lin Accel: x={:.2} m/s², y={:.2} m/s², z={:.2} m/s²",
                        lin_accel.x, lin_accel.y, lin_accel.z
                    );
                } else {
                    println!("Lin Accel: Not available");
                }

                if let Some(gravity) = data.gravity {
                    println!(
                        "Gravity: x={:.2} m/s², y={:.2} m/s², z={:.2} m/s²",
                        gravity.x, gravity.y, gravity.z
                    );
                } else {
                    println!("Gravity: Not available");
                }

                 if let Some(temp) = data.temperature {
                     println!("Temp:  {:.1} °C", temp);
                } else {
                    println!("Temp: Not available");
                }

                println!("---");
            }
            Err(e) => {
                eprintln!("Error reading IMU data: {}", e);
            }
        }

        thread::sleep(Duration::from_millis(100));
    }
}
