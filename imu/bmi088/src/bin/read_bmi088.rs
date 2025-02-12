use linux_bmi088::Bmi088Reader;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let imu = Bmi088Reader::new("/dev/i2c-1")?;
    println!("Reading BMI088 sensor data...");

    loop {
        let data = imu.get_data()?;
        println!(
            "Quaternion: w={:.3}, x={:.3}, y={:.3}, z={:.3}",
            data.quaternion.w, data.quaternion.x, data.quaternion.y, data.quaternion.z
        );
        println!(
            "Accel: x={:.2} g, y={:.2} g, z={:.2} g",
            data.accelerometer.x, data.accelerometer.y, data.accelerometer.z
        );
        println!(
            "Gyro:  x={:.2} °/s, y={:.2} °/s, z={:.2} °/s",
            data.gyroscope.x, data.gyroscope.y, data.gyroscope.z
        );
        println!(
            "Euler: roll={:.1}°, pitch={:.1}°, yaw={:.1}°",
            data.euler.roll, data.euler.pitch, data.euler.yaw
        );
        println!("Temp:  {:.1} °C", data.temperature);
        println!("----------------------------------------");
        thread::sleep(Duration::from_millis(100));
    }
}
