use hiwonder::{HiwonderReader, ImuFrequency, ImuReader, Quaternion, Vector3};
use std::io;
use std::thread;
use std::time::Duration;

fn main() -> io::Result<()> {
    let (ports_to_try, baud_rate) = if cfg!(target_os = "linux") {
        (vec!["/dev/ttyUSB0"], 230400)
    } else if cfg!(target_os = "macos") {
        // TODO: This is probably not the best way to do this (can only read
        // at baud rate 9600) but it is useful for debugging the numerical
        // values while on a Mac.
        (vec!["/dev/tty.usbserial-83420"], 9600)
    } else {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("Unsupported OS: {}", std::env::consts::OS),
        ));
    };

    let mut reader = None;
    for port in ports_to_try {
        match HiwonderReader::new(&port, baud_rate) {
            Ok(r) => {
                println!("Successfully connected to {}", port);
                reader = Some(r);
                break;
            }
            Err(_) => {
                eprintln!("Failed to connect to {}", port);
            }
        }
    }

    let reader = match reader {
        Some(r) => r,
        None => {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                "No valid port found",
            ))
        }
    };

    match reader.set_frequency(ImuFrequency::Hz200) {
        Ok(_) => println!("Set frequency to 200hz"),
        Err(e) => return Err(io::Error::new(io::ErrorKind::Other, e.to_string())),
    }

    loop {
        match reader.get_data() {
            Ok(data) => {
                let accel = data.accelerometer.unwrap_or(Vector3::default());
                let gyro = data.gyroscope.unwrap_or(Vector3::default());
                let angle = data.euler.unwrap_or(Vector3::default());
                let quaternion = data.quaternion.unwrap_or(Quaternion::default());
                let magnetometer = data.magnetometer.unwrap_or(Vector3::default());

                // Computes projected gravity from the quaternion.
                let gravity = quaternion.rotate(Vector3::new(0.0, 0.0, -1.0));

                println!(
                    "acc:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     gyro:  x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     angle: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     quaternion: x: {: >10.3} y: {: >10.3} z: {: >10.3} w: {: >10.3}\n\
                     mag:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     temp:  {: >10.3}\n\
                     gravity: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
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
                    gravity.x,
                    gravity.y,
                    gravity.z,
                );
            }
            Err(e) => eprintln!("Error reading from IMU: {}", e),
        }

        thread::sleep(Duration::from_millis(10));
    }
}
