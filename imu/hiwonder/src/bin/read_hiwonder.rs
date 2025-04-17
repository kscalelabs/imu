use hiwonder::{ImuFrequency, HiwonderReader};
use std::io::{self, Write};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH, Instant};
use std::fs::OpenOptions;

fn main() -> io::Result<()> {
    println!("Connecting to IMU device...");
    
    let mut reader =
        HiwonderReader::new("/dev/ttyUSB0", 9600).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    println!("Connected to IMU. Setting frequency...");
    
    match reader.set_frequency(ImuFrequency::Hz200) {
        Ok(_) => println!("Set frequency to 50Hz"),
        Err(e) => println!("Failed to set frequency: {}", e),
    }

    println!("Initializing... please wait");
    thread::sleep(Duration::from_millis(500));  // Wait longer for initialization

    // Record start time
    let start_time = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
    
    // Create or open CSV file
    let mut file = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open(format!("imu_data_{}.csv", start_time.as_secs()))?;
    
    // Write CSV header
    writeln!(
        file,
        "timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z,quat_x,quat_y,quat_z,quat_w"
    )?;

    let mut success_count = 0;
    let mut error_count = 0;
    
    // For maintaining consistent sampling rate
    let target_interval = Duration::from_micros(5000); // 5000 microseconds = 5ms = 200Hz
    let mut last_sample = Instant::now();
    
    loop {
        let loop_start = Instant::now();
        
        match reader.get_data() {
            Ok(data) => {
                // Get current timestamp and offset it with start time
                let current_time = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
                let elapsed_time = current_time.as_secs_f64() - start_time.as_secs_f64();
                
                success_count += 1;
                if success_count % 100 == 0 {
                    println!("Successfully read {} data points", success_count);
                }

                println!("data: {:?}", data);
                
                // Write data to CSV file with offset timestamp
                writeln!(
                    file,
                    "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                    elapsed_time,
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
                    data.quaternion[3]
                )?;
                
                // Reset error count on successful read
                error_count = 0;
                
                // Flush to ensure data is written
                file.flush()?;
            }
            Err(e) => {
                error_count += 1;
                eprintln!("Error reading from IMU: {}", e);
                
                // If we've had many consecutive errors, try resetting
                if error_count > 10 {
                    println!("Too many errors, attempting to reset IMU...");
                    match reader.reset() {
                        Ok(_) => println!("IMU reset attempted"),
                        Err(e) => println!("Failed to reset IMU: {}", e),
                    }
                    error_count = 0;
                    thread::sleep(Duration::from_millis(500));
                }
            }
        }

        // Calculate time spent in this iteration
        let iteration_time = loop_start.elapsed();
        
        // Sleep for the remaining time to maintain 200Hz
        if iteration_time < target_interval {
            thread::sleep(target_interval - iteration_time);
        } else {
            // Log if we're not keeping up with the desired rate
            if iteration_time > target_interval * 2 {
                eprintln!("Warning: Can't maintain 200Hz sampling rate. Iteration took {:?}", iteration_time);
            }
        }
        
        // Update last sample time
        last_sample = Instant::now();
    }
}
