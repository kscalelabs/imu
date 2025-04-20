use linux_bno055::Bno055Reader;
use std::{thread, time::{Duration, Instant}};
use std::time::SystemTime;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a new BNO055 reader
    let imu = Bno055Reader::new("/dev/i2c-1")?;
    
    let target_period = Duration::from_millis(10);
    
    println!("Reading IMU data...");
    println!("Press Ctrl+C to exit");

    let mut last_loop = Instant::now();
    
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
            // Calculate actual loop time
            let loop_duration = loop_start.duration_since(last_loop);
            println!("Loop time: {:?}", loop_duration);
        }

        println!("---");
        
        // Calculate remaining time to sleep
        let elapsed = loop_start.elapsed();
        if elapsed < target_period {
            thread::sleep(target_period - elapsed);
        }
        
        last_loop = loop_start;
    }
}