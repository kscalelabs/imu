use linux_bno055::Bno055Reader;
use std::{thread, time::Duration};
use std::fs::File;
use std::io::Write;
use std::time::{SystemTime, UNIX_EPOCH};
use nalgebra::{UnitQuaternion, Vector3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a new BNO055 reader
    let imu = Bno055Reader::new("/dev/i2c-1")?;
    
    // Create a file to save data
    let mut file = File::create("bno55_imu_data.csv")?;
    
    // Write CSV header
    writeln!(file, "timestamp,quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,lin_accel_x,lin_accel_y,lin_accel_z,grav_x,grav_y,grav_z,proj_grav_x,proj_grav_y,proj_grav_z,temp,calib")?;

    println!("Reading IMU data...");
    println!("Press Ctrl+C to exit");

    loop {
        if let Ok(data) = imu.get_data() {
            // Get current timestamp
            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs_f64();
                
            // Extract quaternion components
            let quat_w = data.quaternion.w;
            let quat_x = data.quaternion.x;
            let quat_y = data.quaternion.y;
            let quat_z = data.quaternion.z;
            
            // Calculate projected gravity using nalgebra
            // Equivalent to Python: r = Rotation.from_quat([quat_w, quat_x, quat_y, quat_z], scalar_first=True)
            let quaternion = UnitQuaternion::from_quaternion(
                nalgebra::Quaternion::new(quat_w, quat_x, quat_y, quat_z)
            );
            
            // Create the gravity vector [0, 0, 1.0]
            let gravity_vector = Vector3::new(0.0, 0.0, 1.0);
            
            // Apply inverse rotation to gravity vector
            // Equivalent to: proj_grav_world = r.apply(np.array([0, 0, 1.0]), inverse=True)
            let projected_gravity = quaternion.inverse() * gravity_vector;
            
            let proj_grav_x = projected_gravity.x;
            let proj_grav_y = projected_gravity.y;
            let proj_grav_z = projected_gravity.z;
            
            // // Print data to console
            // println!("Quaternion: {:?}", data.quaternion);
            // println!("Euler angles: {:?}", data.euler);
            // println!("Accelerometer: {:?}", data.accelerometer);
            // println!("Gyroscope: {:?}", data.gyroscope);
            // println!("Magnetometer: {:?}", data.magnetometer);
            // println!("Linear acceleration: {:?}", data.linear_acceleration);
            // println!("Gravity: {:?}", data.gravity);
            // println!("Projected gravity: ({:.4}, {:.4}, {:.4})", proj_grav_x, proj_grav_y, proj_grav_z);
            // println!("Temperature: {:?}", data.temperature);
            // println!("Calibration status: {:?}", data.calibration_status);
            
            // Save data to file
            writeln!(
                file,
                "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{}",
                timestamp,
                quat_w, quat_x, quat_y, quat_z,
                data.euler.roll, data.euler.pitch, data.euler.yaw,
                data.accelerometer.x, data.accelerometer.y, data.accelerometer.z,
                data.gyroscope.x, data.gyroscope.y, data.gyroscope.z,
                data.magnetometer.x, data.magnetometer.y, data.magnetometer.z,
                data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
                data.gravity.x, data.gravity.y, data.gravity.z,
                proj_grav_x, proj_grav_y, proj_grav_z,
                data.temperature, data.calibration_status
            )?;
        }

        // println!("---");
        thread::sleep(Duration::from_millis(3));
    }
}
