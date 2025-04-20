use linux_bno055::Bno055Reader;
use std::{thread, time::{Duration, Instant}};
use std::time::SystemTime;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a new BNO055 reader
    let imu = Bno055Reader::new("/dev/i2c-1")?;

    let target_period = Duration::from_millis(10);

    println!("Reading IMU data...");
    println!("Press Ctrl+C to exit");

    let start_time = Instant::now();
    let mut total_steps = 0;
    let mut next_time = Instant::now();

    loop {
        let loop_start = Instant::now();

        if let Ok(data) = imu.get_data() {
            let timestamp = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap();

            println!("Timestamp: {}.{:09} seconds",
                    timestamp.as_secs(),
                    timestamp.subsec_nanos());

            //println!("Quaternion: {:?}", data.quaternion);
            //println!("Euler angles: {:?}", data.euler);
            println!("Accelerometer: {:?}", data.accelerometer);
            println!("Gyroscope: {:?}", data.gyroscope);
        }

        total_steps += 1;
        let steps_per_second = total_steps as f64 / (loop_start.duration_since(start_time).as_secs_f64());
        println!("Steps per second: {:.2}", steps_per_second);

        println!("---");

        next_time += target_period;
        let sleep_duration = next_time.checked_duration_since(Instant::now()).unwrap_or(Duration::new(0, 0));
        if sleep_duration > Duration::new(0, 0) {
            thread::sleep(sleep_duration);
        } else {
            println!("Warning: Loop took longer than target period!");
        }
    }
}
