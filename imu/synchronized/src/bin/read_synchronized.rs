use hiwonder::{HiwonderReader, ImuFrequency};
use linux_bno055::Bno055Reader;
use nalgebra::{UnitQuaternion, Vector3};
use std::fs::File;
use std::io::{self, Write};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

struct ImuData {
    timestamp: f64,
    hiwonder_data: Option<HiwonderData>,
    bno055_data: Option<Bno055Data>,
}

struct HiwonderData {
    accelerometer: [f32; 3],
    gyroscope: [f32; 3],
    angle: [f32; 3],
    quaternion: [f32; 4],
    proj_grav: [f32; 3],
}

struct Bno055Data {
    quaternion: [f32; 4],
    euler: [f32; 3],
    accelerometer: [f32; 3],
    gyroscope: [f32; 3],
    magnetometer: [f32; 3],
    linear_acceleration: [f32; 3],
    gravity: [f32; 3],
    proj_grav: [f32; 3],
    temperature: i8,
    calibration_status: u8,
}

fn calculate_projected_gravity(quat_w: f32, quat_x: f32, quat_y: f32, quat_z: f32) -> [f32; 3] {
    // Calculate projected gravity using nalgebra
    let quaternion = UnitQuaternion::from_quaternion(
        nalgebra::Quaternion::new(quat_w as f64, quat_x as f64, quat_y as f64, quat_z as f64)
    );
    
    // Create the gravity vector [0, 0, 1.0]
    let gravity_vector = Vector3::new(0.0, 0.0, 1.0);
    
    // Apply inverse rotation to gravity vector
    let projected_gravity = quaternion.inverse() * gravity_vector;
    
    [
        projected_gravity.x as f32,
        projected_gravity.y as f32,
        projected_gravity.z as f32
    ]
}

fn main() -> io::Result<()> {
    println!("Initializing IMU sensors...");
    
    // Initialize the BNO055 sensor
    let bno055 = match Bno055Reader::new("/dev/i2c-1") {
        Ok(reader) => {
            println!("BNO055 sensor initialized successfully");
            Some(reader)
        },
        Err(e) => {
            eprintln!("Failed to initialize BNO055 sensor: {}", e);
            None
        }
    };

    // Initialize the Hiwonder sensor
    let hiwonder = match HiwonderReader::new("/dev/ttyUSB0", 9600) {
        Ok(reader) => {
            println!("Hiwonder sensor initialized successfully");
            
            // Set the frequency to 200Hz
            if let Err(e) = reader.set_frequency(ImuFrequency::Hz200) {
                eprintln!("Warning: Failed to set Hiwonder frequency: {}", e);
            } else {
                println!("Hiwonder frequency set to 200Hz");
            }
            
            Some(reader)
        },
        Err(e) => {
            eprintln!("Failed to initialize Hiwonder sensor: {}", e);
            None
        }
    };

    // Check if at least one sensor is available
    if bno055.is_none() && hiwonder.is_none() {
        return Err(io::Error::new(io::ErrorKind::Other, "No IMU sensors available"));
    }

    // Create a file to save synchronized data
    let mut file = File::create("synchronized_imu_data.csv")?;
    
    // Write CSV header
    writeln!(
        file, 
        "timestamp,\
        hw_acc_x,hw_acc_y,hw_acc_z,\
        hw_gyro_x,hw_gyro_y,hw_gyro_z,\
        hw_angle_x,hw_angle_y,hw_angle_z,\
        hw_quat_x,hw_quat_y,hw_quat_z,hw_quat_w,\
        hw_proj_grav_x,hw_proj_grav_y,hw_proj_grav_z,\
        bno_quat_w,bno_quat_x,bno_quat_y,bno_quat_z,\
        bno_roll,bno_pitch,bno_yaw,\
        bno_acc_x,bno_acc_y,bno_acc_z,\
        bno_gyro_x,bno_gyro_y,bno_gyro_z,\
        bno_mag_x,bno_mag_y,bno_mag_z,\
        bno_lin_acc_x,bno_lin_acc_y,bno_lin_acc_z,\
        bno_grav_x,bno_grav_y,bno_grav_z,\
        bno_proj_grav_x,bno_proj_grav_y,bno_proj_grav_z,\
        bno_temp,bno_calib"
    )?;

    println!("Reading synchronized IMU data...");
    println!("Press Ctrl+C to exit");

    // Main loop for reading data from both sensors
    loop {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        
        let mut imu_data = ImuData {
            timestamp,
            hiwonder_data: None,
            bno055_data: None,
        };

        // Read data from Hiwonder sensor if available
        if let Some(ref reader) = hiwonder {
            match reader.get_data() {
                Ok(data) => {
                    // Extract quaternion components
                    let quat_x = data.quaternion[0];
                    let quat_y = data.quaternion[1];
                    let quat_z = data.quaternion[2];
                    let quat_w = data.quaternion[3];
                    
                    // Calculate projected gravity
                    let proj_grav = calculate_projected_gravity(quat_w, quat_x, quat_y, quat_z);
                    
                    imu_data.hiwonder_data = Some(HiwonderData {
                        accelerometer: data.accelerometer,
                        gyroscope: data.gyroscope,
                        angle: data.angle,
                        quaternion: data.quaternion,
                        proj_grav,
                    });
                }
                Err(e) => eprintln!("Error reading from Hiwonder IMU: {}", e),
            }
        }

        // Read data from BNO055 sensor if available
        if let Some(ref imu) = bno055 {
            match imu.get_data() {
                Ok(data) => {
                    // Extract quaternion components
                    let quat_w = data.quaternion.w;
                    let quat_x = data.quaternion.x;
                    let quat_y = data.quaternion.y;
                    let quat_z = data.quaternion.z;
                    
                    // Calculate projected gravity
                    let proj_grav = calculate_projected_gravity(quat_w, quat_x, quat_y, quat_z);
                    
                    imu_data.bno055_data = Some(Bno055Data {
                        quaternion: [quat_w, quat_x, quat_y, quat_z],
                        euler: [data.euler.roll, data.euler.pitch, data.euler.yaw],
                        accelerometer: [data.accelerometer.x, data.accelerometer.y, data.accelerometer.z],
                        gyroscope: [data.gyroscope.x, data.gyroscope.y, data.gyroscope.z],
                        magnetometer: [data.magnetometer.x, data.magnetometer.y, data.magnetometer.z],
                        linear_acceleration: [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z],
                        gravity: [data.gravity.x, data.gravity.y, data.gravity.z],
                        proj_grav,
                        temperature: data.temperature,
                        calibration_status: data.calibration_status,
                    });
                }
                Err(e) => eprintln!("Error reading from BNO055 IMU: {}", e),
            }
        }

        // Write data to console
        println!("=== Synchronized IMU Data ===");
        println!("Timestamp: {:.6}", imu_data.timestamp);
        
        if let Some(hw_data) = &imu_data.hiwonder_data {
            println!("Hiwonder:");
            println!("  Accel: [{:.3}, {:.3}, {:.3}]", 
                hw_data.accelerometer[0], hw_data.accelerometer[1], hw_data.accelerometer[2]);
            println!("  Gyro:  [{:.3}, {:.3}, {:.3}]", 
                hw_data.gyroscope[0], hw_data.gyroscope[1], hw_data.gyroscope[2]);
            println!("  Angle: [{:.3}, {:.3}, {:.3}]", 
                hw_data.angle[0], hw_data.angle[1], hw_data.angle[2]);
            println!("  Quat:  [{:.3}, {:.3}, {:.3}, {:.3}]", 
                hw_data.quaternion[0], hw_data.quaternion[1], hw_data.quaternion[2], hw_data.quaternion[3]);
            println!("  Proj Grav: [{:.3}, {:.3}, {:.3}]",
                hw_data.proj_grav[0], hw_data.proj_grav[1], hw_data.proj_grav[2]);
        }
        
        if let Some(bno_data) = &imu_data.bno055_data {
            println!("BNO055:");
            println!("  Quat:  [{:.3}, {:.3}, {:.3}, {:.3}]", 
                bno_data.quaternion[0], bno_data.quaternion[1], bno_data.quaternion[2], bno_data.quaternion[3]);
            println!("  Euler: [{:.3}, {:.3}, {:.3}]", 
                bno_data.euler[0], bno_data.euler[1], bno_data.euler[2]);
            println!("  Accel: [{:.3}, {:.3}, {:.3}]", 
                bno_data.accelerometer[0], bno_data.accelerometer[1], bno_data.accelerometer[2]);
            println!("  Proj Grav: [{:.3}, {:.3}, {:.3}]",
                bno_data.proj_grav[0], bno_data.proj_grav[1], bno_data.proj_grav[2]);
        }
        
        // Write synchronized data to CSV file
        let hw_data = imu_data.hiwonder_data.as_ref();
        let bno_data = imu_data.bno055_data.as_ref();
        
        writeln!(
            file,
            "{:.6},\
            {},{},{},\
            {},{},{},\
            {},{},{},\
            {},{},{},{},\
            {},{},{},\
            {},{},{},{},\
            {},{},{},\
            {},{},{},\
            {},{},{},\
            {},{},{},\
            {},{},{},\
            {},{},{},\
            {},{},{},\
            {},{}",
            imu_data.timestamp,
            // Hiwonder data
            hw_data.map_or(String::from(""), |d| d.accelerometer[0].to_string()),
            hw_data.map_or(String::from(""), |d| d.accelerometer[1].to_string()),
            hw_data.map_or(String::from(""), |d| d.accelerometer[2].to_string()),
            hw_data.map_or(String::from(""), |d| d.gyroscope[0].to_string()),
            hw_data.map_or(String::from(""), |d| d.gyroscope[1].to_string()),
            hw_data.map_or(String::from(""), |d| d.gyroscope[2].to_string()),
            hw_data.map_or(String::from(""), |d| d.angle[0].to_string()),
            hw_data.map_or(String::from(""), |d| d.angle[1].to_string()),
            hw_data.map_or(String::from(""), |d| d.angle[2].to_string()),
            hw_data.map_or(String::from(""), |d| d.quaternion[0].to_string()),
            hw_data.map_or(String::from(""), |d| d.quaternion[1].to_string()),
            hw_data.map_or(String::from(""), |d| d.quaternion[2].to_string()),
            hw_data.map_or(String::from(""), |d| d.quaternion[3].to_string()),
            hw_data.map_or(String::from(""), |d| d.proj_grav[0].to_string()),
            hw_data.map_or(String::from(""), |d| d.proj_grav[1].to_string()),
            hw_data.map_or(String::from(""), |d| d.proj_grav[2].to_string()),
            // BNO055 data
            bno_data.map_or(String::from(""), |d| d.quaternion[0].to_string()),
            bno_data.map_or(String::from(""), |d| d.quaternion[1].to_string()),
            bno_data.map_or(String::from(""), |d| d.quaternion[2].to_string()),
            bno_data.map_or(String::from(""), |d| d.quaternion[3].to_string()),
            bno_data.map_or(String::from(""), |d| d.euler[0].to_string()),
            bno_data.map_or(String::from(""), |d| d.euler[1].to_string()),
            bno_data.map_or(String::from(""), |d| d.euler[2].to_string()),
            bno_data.map_or(String::from(""), |d| d.accelerometer[0].to_string()),
            bno_data.map_or(String::from(""), |d| d.accelerometer[1].to_string()),
            bno_data.map_or(String::from(""), |d| d.accelerometer[2].to_string()),
            bno_data.map_or(String::from(""), |d| d.gyroscope[0].to_string()),
            bno_data.map_or(String::from(""), |d| d.gyroscope[1].to_string()),
            bno_data.map_or(String::from(""), |d| d.gyroscope[2].to_string()),
            bno_data.map_or(String::from(""), |d| d.magnetometer[0].to_string()),
            bno_data.map_or(String::from(""), |d| d.magnetometer[1].to_string()),
            bno_data.map_or(String::from(""), |d| d.magnetometer[2].to_string()),
            bno_data.map_or(String::from(""), |d| d.linear_acceleration[0].to_string()),
            bno_data.map_or(String::from(""), |d| d.linear_acceleration[1].to_string()),
            bno_data.map_or(String::from(""), |d| d.linear_acceleration[2].to_string()),
            bno_data.map_or(String::from(""), |d| d.gravity[0].to_string()),
            bno_data.map_or(String::from(""), |d| d.gravity[1].to_string()),
            bno_data.map_or(String::from(""), |d| d.gravity[2].to_string()),
            bno_data.map_or(String::from(""), |d| d.proj_grav[0].to_string()),
            bno_data.map_or(String::from(""), |d| d.proj_grav[1].to_string()),
            bno_data.map_or(String::from(""), |d| d.proj_grav[2].to_string()),
            bno_data.map_or(String::from(""), |d| d.temperature.to_string()),
            bno_data.map_or(String::from(""), |d| d.calibration_status.to_string())
        )?;

        // Sleep for a short duration to synchronize data collection
        // thread::sleep(Duration::from_millis(5));
    }
} 