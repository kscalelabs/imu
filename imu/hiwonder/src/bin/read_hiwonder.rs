use hiwonder::{HiwonderReader, ImuFrequency};
use std::io;
use std::thread;
use std::time::{Duration, Instant, SystemTime};

#[derive(Debug, Copy, Clone)]
struct LastReading {
    accel: [f32; 3],
    gyro: [f32; 3],
    duplicates: u32,
}

#[derive(Debug)]
struct Stats {
    total_readings: u64,
    total_duplicates: u64,
    start_time: Instant,
}

impl Stats {
    fn new() -> Self {
        Stats {
            total_readings: 0,
            total_duplicates: 0,
            start_time: Instant::now(),
        }
    }

    fn print_summary(&self) {
        let runtime = self.start_time.elapsed().as_secs_f64();
        
        let duplicate_percent = if self.total_readings > 0 {
            (self.total_duplicates as f64 / self.total_readings as f64) * 100.0
        } else {
            0.0
        };
        
        let raw_rate = self.total_readings as f64 / runtime;
        let effective_hz = (self.total_readings - self.total_duplicates) as f64 / runtime;
        
        println!(
            "Stats: {} duplicates/{} total readings ({:.1}%) over {:.1}s - Effective rate: {:.1} Hz (Raw: {:.1} Hz)",
            self.total_duplicates,
            self.total_readings,
            duplicate_percent,
            runtime,
            effective_hz,
            raw_rate
        );
    }
}

fn main() -> io::Result<()> {
    let reader = HiwonderReader::new("/dev/ttyUSB0", 230400)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    match reader.set_frequency(ImuFrequency::Hz100) {
        Ok(_) => println!("Set frequency to 100hz"),
        Err(e) => println!("Failed to set frequency: {}", e),
    }

    let mut last_reading = None::<LastReading>;
    let mut stats = Stats::new();

    loop {
        match reader.get_data() {
            Ok(data) => {
                stats.total_readings += 1;
                
                let timestamp = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap();

                // Check for duplicates
                let is_duplicate = if let Some(last) = last_reading {
                    data.accelerometer == last.accel && data.gyroscope == last.gyro
                } else {
                    false
                };

                if is_duplicate {
                    stats.total_duplicates += 1;
                }

                let duplicate_marker = if is_duplicate {
                    if let Some(last) = last_reading {
                        format!(" DUPLICATE({})", last.duplicates + 1)
                    } else {
                        String::new()
                    }
                } else {
                    String::new()
                };
                
                /*println!(
                    "timestamp: {}.{:09}{}\n\
                     acc:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     gyro:  x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     angle: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     quaternion: x: {: >10.3} y: {: >10.3} z: {: >10.3} w: {: >10.3}\n\
                     mag:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     temp:  {: >10.3}\n\
                     ",
                    timestamp.as_secs(),
                    timestamp.subsec_nanos(),
                    duplicate_marker,
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
                );*/

                // Print running statistics every 100 readings
                if stats.total_readings % 100 == 0 {
                    stats.print_summary();
                }

                // Update last reading
                last_reading = Some(LastReading {
                    accel: data.accelerometer,
                    gyro: data.gyroscope,
                    duplicates: if is_duplicate {
                        last_reading.map_or(0, |last| last.duplicates + 1)
                    } else {
                        0
                    },
                });
            }
            Err(e) => eprintln!("Error reading from IMU: {}", e),
        }

        thread::sleep(Duration::from_millis(10));
    }
}