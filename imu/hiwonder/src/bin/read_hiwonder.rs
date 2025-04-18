use hiwonder::{HiwonderReader, ImuFrequency};
use std::io;
use std::thread;
use std::time::Duration;

fn main() -> io::Result<()> {
    let reader = HiwonderReader::new("/dev/ttyUSB0", 9600)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    match reader.set_frequency(ImuFrequency::Hz200) {
        Ok(_) => println!("Set frequency to 200hz"),
        Err(e) => println!("Failed to set frequency: {}", e),
    }

    loop {
        match reader.get_data() {
            Ok(data) => {
                println!(
                    "acc:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     gyro:  x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     angle: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     quaternion: x: {: >10.3} y: {: >10.3} z: {: >10.3} w: {: >10.3}\n\
                     mag:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     temp:  {: >10.3}\n\
                     ",
                    data.accelerometer[0],
                    data.accelerometer[1],
                    data.accelerometer[2],
                    data.gyroscope[0],
                    data.gyroscope[1],
                    data.gyroscope[2],
                    data.angle[0],
                    data.angle[1],
                    data.angle[2],
                    data.quaternion[0],
                    data.quaternion[1],
                    data.quaternion[2],
                    data.quaternion[3],
                    data.magnetometer[0],
                    data.magnetometer[1],
                    data.magnetometer[2],
                    data.temperature,
                );
            }
            Err(e) => eprintln!("Error reading from IMU: {}", e),
        }

        thread::sleep(Duration::from_millis(10));
    }
}
