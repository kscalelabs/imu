use linux_bno055::Bno055Reader;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a new BNO055 reader
    let imu = Bno055Reader::new("/dev/i2c-1")?;

    println!("Reading IMU data...");
    println!("Press Ctrl+C to exit");

    loop {
        if let Ok(data) = imu.get_data() {
            println!("Quaternion: {:?}", data.quaternion);
            println!("Euler angles: {:?}", data.euler);
            println!("Accelerometer: {:?}", data.accelerometer);
            // Print other data as necessary
        }

        println!("---");
        thread::sleep(Duration::from_millis(100));
    }
}
