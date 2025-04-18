use linux_bmi088::{Bmi088Reader, ImuReader, Vector3};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let imu = Bmi088Reader::new("/dev/i2c-1")?;
    println!("Reading BMI088 sensor data...");

    loop {
        let data = imu.get_data()?;
        let accel = data.accelerometer.unwrap_or(Vector3::default());
        let gyro = data.gyroscope.unwrap_or(Vector3::default());
        println!(
            "Accel: x={:.2} g, y={:.2} g, z={:.2} g",
            accel.x, accel.y, accel.z
        );
        println!(
            "Gyro:  x={:.2} °/s, y={:.2} °/s, z={:.2} °/s",
            gyro.x, gyro.y, gyro.z
        );
        println!("Temp:  {:.1} °C", data.temperature.unwrap_or(0.0));
        println!("----------------------------------------");
        thread::sleep(Duration::from_millis(100));
    }
}
