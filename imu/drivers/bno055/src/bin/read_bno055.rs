use linux_bno055::{Bno055Reader, ImuReader, ImuData, Vector3, Quaternion};
use std::time::{Duration, Instant};
use std::thread;
use std::collections::HashMap;

#[derive(Debug, Clone)]
struct SensorFrame {
    accelerometer: Option<Vector3>,
    gyroscope: Option<Vector3>,
    quaternion: Option<Quaternion>,
    //euler: Option<Vector3>,
    gravity: Option<Vector3>,
    //linear_acceleration: Option<Vector3>,
    //magnetometer: Option<Vector3>,
    //temperature: Option<f32>,
    calibration_status: Option<u8>,
}

impl SensorFrame {
    fn from_data(data: &ImuData) -> Self {
        SensorFrame {
            accelerometer: data.accelerometer,
            gyroscope: data.gyroscope,
            quaternion: data.quaternion,
            //euler: data.euler,
            gravity: data.gravity,
            //linear_acceleration: data.linear_acceleration,
            //magnetometer: data.magnetometer,
            //temperature: data.temperature,
            calibration_status: data.calibration_status,
        }
    }

    fn is_duplicate(&self, other: &SensorFrame) -> (bool, Vec<&str>) {
        let mut changed_sensors = vec![];
    
        macro_rules! check {
            ($field:ident, $label:expr) => {
                if self.$field != other.$field {
                    changed_sensors.push($label);
                }
            };
        }
    
        check!(accelerometer, "accel");
        check!(gyroscope, "gyro");
        check!(quaternion, "quat");
        check!(gravity, "gravity");
        check!(calibration_status, "calib");
    
        let is_duplicate = changed_sensors.is_empty();
        (is_duplicate, changed_sensors)
    }
    
}

#[derive(Debug)]
struct Stats {
    total_readings: u64,
    unique_readings: u64,
    sensor_changes: HashMap<String, u64>,
    missed_deadlines: u64,
    start_time: Instant,
}

impl Stats {
    fn new() -> Self {
        let mut sensor_changes = HashMap::new();
        /*for sensor in &[
            "accel", "gyro", "quat", "euler", "gravity", 
            "linear_accel", "mag", "temp", "calib"
        ] {*/
        for sensor in &[
            "accel", "gyro", "quat", "gravity"
        ] {
            sensor_changes.insert((*sensor).to_string(), 0);
        }

        Stats {
            total_readings: 0,
            unique_readings: 0,
            sensor_changes,
            missed_deadlines: 0,
            start_time: Instant::now(),
        }
    }
}

fn decode_calib_status(byte: u8) -> (u8, u8, u8, u8) {
    let sys = (byte >> 6) & 0b11;
    let gyro = (byte >> 4) & 0b11;
    let accel = (byte >> 2) & 0b11;
    let mag = byte & 0b11;
    (sys, gyro, accel, mag)
}


fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a new BNO055 reader
    let imu = Bno055Reader::new("/dev/i2c-1")?;

    println!("Starting IMU readings at 100Hz...");
    println!("Press Ctrl+C to exit\n");

    let mut stats = Stats::new();
    let mut prev_frame: Option<SensorFrame> = None;

    // Set up timing control
    let hz = 100.0;
    let period = Duration::from_secs_f64(1.0 / hz);
    let mut next_time = Instant::now() + period;

    loop {
        if let Ok(data) = imu.get_data() {
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
                "acc:   {:?}\n\
                gyro:  {:?}\n\
                quat:  {:?}\n\
                gravity: {:?}",
                current_frame.accelerometer,
                current_frame.gyroscope,
                current_frame.quaternion,
                current_frame.gravity,
            );
            
            match current_frame.calibration_status {
                Some(status) => {
                    let (sys, gyro, accel, mag) = decode_calib_status(status);
                    println!("calib: sys={}, gyro={}, accel={}, mag={}", sys, gyro, accel, mag);
                }
                None => println!("calib: unavailable"),
            }
            
            println!(
                "Duplicate: {} {}\n\
                Rate: {:.1} Hz effective ({:.1} Hz raw)\n\
                Missed deadlines: {}\n",
                is_duplicate,
                if !is_duplicate {
                    format!("(changed: {})", changed_sensors.join(", "))
                } else {
                    String::new()
                },
                effective_rate,
                raw_rate,
                stats.missed_deadlines
            );
            
        }

        // Loop Timing Control with Drift Correction
        let now = Instant::now();
        if next_time > now {
            thread::sleep(next_time - now);
        } else {
            // We're behind schedule
            eprintln!("⚠️ Missed target by {:?}", now - next_time);
            stats.missed_deadlines += 1;
        }
        next_time += period;
    }
}