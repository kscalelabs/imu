use hiwonder::{HiwonderReader, ImuFrequency};
use std::io;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let reader = HiwonderReader::new("/dev/ttyUSB0", 9600)?;

    match reader.set_frequency(HiwonderImuFrequency::Hz200) {
        Ok(_) => println!("Set frequency to 200hz"),
        Err(e) => println!("Failed to set frequency: {}", e),
    }

    loop {
        match reader.get_data() {
            Ok(data) => {
                print!("acc:   ");
                if let Some(v) = data.accelerometer { print!("x: {: >10.3} y: {: >10.3} z: {: >10.3}", v.x, v.y, v.z) }
                else { print!("Not available") }
                println!();

                print!("gyro:  ");
                if let Some(v) = data.gyroscope { print!("x: {: >10.3} y: {: >10.3} z: {: >10.3}", v.x, v.y, v.z) }
                else { print!("Not available") }
                println!();

                print!("angle: ");
                if let Some(v) = data.euler { print!("x: {: >10.3} y: {: >10.3} z: {: >10.3}", v.x, v.y, v.z) }
                else { print!("Not available") }
                println!();

                print!("quat:  ");
                if let Some(q) = data.quaternion { print!("w: {: >10.3} x: {: >10.3} y: {: >10.3} z: {: >10.3}", q.w, q.x, q.y, q.z) }
                else { print!("Not available") }
                println!();

                print!("mag:   ");
                if let Some(v) = data.magnetometer { print!("x: {: >10.3} y: {: >10.3} z: {: >10.3}", v.x, v.y, v.z) }
                else { print!("Not available") }
                println!();

                print!("temp:  ");
                if let Some(t) = data.temperature { print!("{: >10.3}", t) }
                else { print!("Not available") }
                println!("\n----------------------------------------");
            }
            Err(e) => eprintln!("Error reading from IMU: {}", e),
        }

        thread::sleep(Duration::from_millis(10));
    }
}
