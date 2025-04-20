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

    fn get_stats(&self) -> (f64, f64, f64) {
        let runtime = self.start_time.elapsed().as_secs_f64();
        let duplicate_percent = if self.total_readings > 0 {
            (self.total_duplicates as f64 / self.total_readings as f64) * 100.0
        } else {
            0.0
        };
        let raw_rate = self.total_readings as f64 / runtime;
        let effective_hz = (self.total_readings - self.total_duplicates) as f64 / runtime;
        
        (duplicate_percent, effective_hz, raw_hz)
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

                let (dup_percent, eff_hz, raw_hz) = stats.get_stats();
                
                println!(
                    "t: {}.{:09}{} | acc: [{: >8.3}, {: >8.3}, {: >8.3}] | gyro: [{: >8.3}, {: >8.3}, {: >8.3}] | dups: {}/{} ({:.1}%) | eff_hz: {:.1} (raw: {:.1})",
                    timestamp.as_secs(),
                    timestamp.subsec_nanos(),
                    duplicate_marker,
                    data.accelerometer[0],
                    data.accelerometer[1],
                    data.accelerometer[2],
                    data.gyroscope[0],
                    data.gyroscope[1],
                    data.gyroscope[2],
                    stats.total_duplicates,
                    stats.total_readings,
                    dup_percent,
                    eff_hz,
                    raw_hz
                );

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