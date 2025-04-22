use hiwonder::{HiwonderReader, ImuReader, Quaternion, Vector3};
use std::io;
use std::thread;
use std::time::{Duration, Instant};

fn main() -> io::Result<()> {
    let (ports_to_try, baud_rate) = if cfg!(target_os = "linux") {
        (vec!["/dev/ttyUSB0"], 230400)
    } else if cfg!(target_os = "macos") {
        // TODO: This is probably not the best way to do this (can only read
        // at baud rate 9600) but it is useful for debugging the numerical
        // values while on a Mac.
        (vec!["/dev/tty.usbserial-110"], 10000)
    } else {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("Unsupported OS: {}", std::env::consts::OS),
        ));
    };

    let mut reader = None;
    for port in ports_to_try {
        match HiwonderReader::new(port, baud_rate) {
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

    let mut num_steps = 0;
    let start_time = Instant::now();
    let mut prev_gyro: Option<Vector3> = None;

    loop {
        match reader.get_data() {
            Ok(data) => {
                let current_gyro = data.gyroscope;

                let mut gyro_changed = false;
                if let Some(cg) = current_gyro {
                    if let Some(pg) = prev_gyro {
                        if cg != pg {
                            gyro_changed = true;
                        }
                    } else {
                        gyro_changed = true;
                    }
                } else if prev_gyro.is_some() {
                    gyro_changed = true;
                }

                if gyro_changed {
                    num_steps += 1;
                }

                prev_gyro = current_gyro;

                let accel = data.accelerometer.unwrap_or(Vector3::default());
                let gyro_for_print = current_gyro.unwrap_or(Vector3::default());
                let angle = data.euler.unwrap_or(Vector3::default());
                let quaternion = data.quaternion.unwrap_or(Quaternion::default());
                let magnetometer = data.magnetometer.unwrap_or(Vector3::default());
                let gravity = quaternion.rotate(Vector3::new(0.0, 0.0, -1.0));

                println!(
                    "acc:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     gyro:  x: {: >10.3} y: {: >10.3} z: {: >10.3} (Changed: {})\n\
                     angle: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     quaternion: x: {: >10.3} y: {: >10.3} z: {: >10.3} w: {: >10.3}\n\
                     mag:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     temp:  {: >10.3}\n\
                     gravity: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     num_steps (gyro changed): {: >10}\n\
                     steps/sec: {: >10.2}\n\
                     ",
                    accel.x,
                    accel.y,
                    accel.z,
                    gyro_for_print.x,
                    gyro_for_print.y,
                    gyro_for_print.z,
                    gyro_changed,
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
                    num_steps,
                    num_steps as f32 / start_time.elapsed().as_secs_f32(),
                );
            }
            Err(e) if e.to_string().contains("No new data available") => {}
            Err(e) => {
                eprintln!("Failed to read data: {}", e);
            }
        }

        thread::sleep(Duration::from_millis(10));
    }
}
