use hiwonder::{HiwonderReader, ImuFrequency, ImuReader, Quaternion, Vector3};
use std::io;
use std::thread;
use std::time::Duration;

fn main() -> io::Result<()> {
    let reader = HiwonderReader::new("/dev/ttyUSB0", 230400)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    match reader.set_frequency(ImuFrequency::Hz200) {
        Ok(_) => println!("Set frequency to 200hz"),
        Err(e) => println!("Failed to set frequency: {}", e),
    }

    loop {
        match reader.get_data() {
            Ok(data) => {
                let accel = data.accelerometer.unwrap_or(Vector3::default());
                let gyro = data.gyroscope.unwrap_or(Vector3::default());
                let angle = data.euler.unwrap_or(Vector3::default());
                let quaternion = data.quaternion.unwrap_or(Quaternion::default());
                let magnetometer = data.magnetometer.unwrap_or(Vector3::default());
                println!(
                    "acc:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     gyro:  x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     angle: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     quaternion: x: {: >10.3} y: {: >10.3} z: {: >10.3} w: {: >10.3}\n\
                     mag:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     temp:  {: >10.3}\n\
                     ",
                    accel.x,
                    accel.y,
                    accel.z,
                    gyro.x,
                    gyro.y,
                    gyro.z,
                    angle.x,
                    angle.y,
                    angle.z,
                    quaternion.x,
                    quaternion.y,
                    quaternion.z,
                    quaternion.w,
                    magnetometer.x,
                    magnetometer.y,
                    magnetometer.z,
                    data.temperature.unwrap_or(0.0),
                );
            }
            Err(e) => eprintln!("Error reading from IMU: {}", e),
        }

        thread::sleep(Duration::from_millis(10));
    }
}
