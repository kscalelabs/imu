use clap::Parser;
use hiwonder::{HiwonderReader, ImuReader, Quaternion, Vector3};
use std::io;
use std::thread::sleep;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    #[clap(short, long, default_value = "/dev/ttyUSB0")]
    device: String,

    #[clap(short, long, default_value_t = 230400)]
    baud_rate: u32,
}

#[derive(Debug, Clone)]
struct SensorFrame {
    accelerometer: Vector3,
    gyroscope: Vector3,
    euler: Vector3,
    quaternion: Quaternion,
    magnetometer: Vector3,
    temperature: f32,
}

impl SensorFrame {
    fn from_data(data: &hiwonder::ImuData) -> Self {
        SensorFrame {
            accelerometer: data.accelerometer.unwrap_or_default(),
            gyroscope: data.gyroscope.unwrap_or_default(),
            euler: data.euler.unwrap_or_default(),
            quaternion: data.quaternion.unwrap_or_default(),
            magnetometer: data.magnetometer.unwrap_or_default(),
            temperature: data.temperature.unwrap_or_default(),
        }
    }

    fn is_duplicate(&self, other: &SensorFrame) -> (bool, Vec<&str>) {
        let mut changed_sensors = Vec::new();

        // Check each sensor independently
        if self.accelerometer != other.accelerometer {
            changed_sensors.push("accel");
        }
        if self.gyroscope != other.gyroscope {
            changed_sensors.push("gyro");
        }
        if self.euler != other.euler {
            changed_sensors.push("euler");
        }
        if self.quaternion != other.quaternion {
            changed_sensors.push("quat");
        }
        if self.magnetometer != other.magnetometer {
            changed_sensors.push("mag");
        }
        if self.temperature != other.temperature {
            changed_sensors.push("temp");
        }

        // If any sensor changed, it's not a duplicate
        (changed_sensors.is_empty(), changed_sensors)
    }
}

#[derive(Debug)]
struct Stats {
    total_readings: u64,
    unique_readings: u64,
    sensor_changes: std::collections::HashMap<String, u64>,
    missed_deadlines: u64,
    start_time: Instant,
}

impl Stats {
    fn new() -> Self {
        let mut sensor_changes = std::collections::HashMap::new();
        sensor_changes.insert("accel".to_string(), 0);
        sensor_changes.insert("gyro".to_string(), 0);
        sensor_changes.insert("euler".to_string(), 0);
        sensor_changes.insert("quat".to_string(), 0);
        sensor_changes.insert("mag".to_string(), 0);
        sensor_changes.insert("temp".to_string(), 0);

        Stats {
            total_readings: 0,
            unique_readings: 0,
            sensor_changes,
            missed_deadlines: 0,
            start_time: Instant::now(),
        }
    }
}

fn main() -> io::Result<()> {
    let args = Args::parse();
    let (port, baud_rate) = (args.device, args.baud_rate);

    println!("Attempting to connect to {} at {} baud...", port, baud_rate);
    let reader = match HiwonderReader::new(&port, baud_rate) {
        Ok(r) => {
            println!("Successfully connected to {}", port);
            r
        }
        Err(e) => {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("Failed to connect to {}: {}", port, e),
            ))
        }
    };

    let mut stats = Stats::new();
    let mut prev_frame: Option<SensorFrame> = None;

    println!("Starting IMU readings at 100Hz...");
    println!("Press Ctrl+C to exit\n");

    let hz = 100.0;
    let period = Duration::from_secs_f64(1.0 / hz);
    let mut next_time = Instant::now() + period;

    loop {
        match reader.get_data() {
            Ok(data) => {
                stats.total_readings += 1;
                let current_frame = SensorFrame::from_data(&data);

                // Check for unique reading and track which sensors changed
                let (is_duplicate, changed_sensors) = if let Some(prev) = &prev_frame {
                    current_frame.is_duplicate(prev)
                } else {
                    (false, vec![])
                };

                if !is_duplicate {
                    stats.unique_readings += 1;
                    // Update change counts for each sensor
                    for sensor in &changed_sensors {
                        // Changed to use reference
                        if let Some(count) = stats.sensor_changes.get_mut(*sensor) {
                            *count += 1;
                        }
                    }
                }

                // Update previous frame
                prev_frame = Some(current_frame.clone());

                // Calculate current frequencies
                let runtime = stats.start_time.elapsed().as_secs_f64();
                let raw_rate = stats.total_readings as f64 / runtime;
                let effective_rate = stats.unique_readings as f64 / runtime;

                println!(
                    "acc:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                    gyro:  x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                    angle: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                    quaternion: x: {: >10.3} y: {: >10.3} z: {: >10.3} w: {: >10.3}\n\
                    mag:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                    temp:  {: >10.3}\n\
                    gravity: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                    Duplicate: {} {}\n\
                    Rate: {:.1} Hz effective ({:.1} Hz raw)\n",
                    current_frame.accelerometer.x,
                    current_frame.accelerometer.y,
                    current_frame.accelerometer.z,
                    current_frame.gyroscope.x,
                    current_frame.gyroscope.y,
                    current_frame.gyroscope.z,
                    current_frame.euler.x,
                    current_frame.euler.y,
                    current_frame.euler.z,
                    current_frame.quaternion.x,
                    current_frame.quaternion.y,
                    current_frame.quaternion.z,
                    current_frame.quaternion.w,
                    current_frame.magnetometer.x,
                    current_frame.magnetometer.y,
                    current_frame.magnetometer.z,
                    current_frame.temperature,
                    data.quaternion
                        .unwrap_or(Quaternion::default())
                        .rotate(Vector3::new(0.0, 0.0, -1.0))
                        .x,
                    data.quaternion
                        .unwrap_or(Quaternion::default())
                        .rotate(Vector3::new(0.0, 0.0, -1.0))
                        .y,
                    data.quaternion
                        .unwrap_or(Quaternion::default())
                        .rotate(Vector3::new(0.0, 0.0, -1.0))
                        .z,
                    is_duplicate,
                    if !is_duplicate {
                        format!("(changed: {})", changed_sensors.join(", "))
                    } else {
                        String::new()
                    },
                    effective_rate,
                    raw_rate
                );
            }
            Err(e) if e.to_string().contains("No new data available") => {}
            Err(e) => {
                eprintln!("Failed to read data: {}", e);
            }
        }

        //Loop Timing Control with Drift Correction
        let now = Instant::now();
        if next_time > now {
            sleep(next_time - now);
        } else {
            // we're behind schedule, don't sleep
            println!("⚠️ Missed target by {:?}", now - next_time);
            stats.missed_deadlines += 1;
        }
        next_time += period;
    }
}
