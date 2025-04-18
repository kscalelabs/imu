use hiwonder::{ImuFrequency, HiwonderReader};
use std::io::{self, Write};
use std::fs::File;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use nalgebra::{UnitQuaternion, Vector3};

fn main() -> io::Result<()> {
    let reader =
        HiwonderReader::new("/dev/ttyUSB0", 9600).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    match reader.set_frequency(ImuFrequency::Hz200) {
        Ok(_) => println!("Set frequency to 200hz"),
        Err(e) => println!("Failed to set frequency: {}", e),
    }

    // Create a file to save data
    let mut file = File::create("hiwonder_imu_data.csv")?;
    
    // Write CSV header
    writeln!(file, "timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z,quat_x,quat_y,quat_z,quat_w,proj_grav_x,proj_grav_y,proj_grav_z")?;

    println!("Reading IMU data...");
    println!("Press Ctrl+C to exit");

    loop {
        match reader.get_data() {
            Ok(data) => {
                // Get current timestamp
                let timestamp = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_secs_f64();
                
                // Extract quaternion components
                let quat_x = data.quaternion[0];
                let quat_y = data.quaternion[1];
                let quat_z = data.quaternion[2];
                let quat_w = data.quaternion[3];
                
                // Calculate projected gravity using nalgebra
                let quaternion = UnitQuaternion::from_quaternion(
                    nalgebra::Quaternion::new(quat_w, quat_x, quat_y, quat_z)
                );
                
                // Create the gravity vector [0, 0, 1.0]
                let gravity_vector = Vector3::new(0.0, 0.0, 1.0);
                
                // Apply inverse rotation to gravity vector
                let projected_gravity = quaternion.inverse() * gravity_vector;
                
                let proj_grav_x = projected_gravity.x;
                let proj_grav_y = projected_gravity.y;
                let proj_grav_z = projected_gravity.z;
                
                // Print to console
                println!(
                    "acc:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     gyro:  x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     angle: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     quaternion: x: {: >10.3} y: {: >10.3} z: {: >10.3} w: {: >10.3}\n\
                     proj_grav: x: {: >10.3} y: {: >10.3} z: {: >10.3}",
                    data.accelerometer[0],
                    data.accelerometer[1],
                    data.accelerometer[2],
                    data.gyroscope[0],
                    data.gyroscope[1],
                    data.gyroscope[2],
                    data.angle[0],
                    data.angle[1],
                    data.angle[2],
                    quat_x,
                    quat_y,
                    quat_z,
                    quat_w,
                    proj_grav_x,
                    proj_grav_y,
                    proj_grav_z
                );

                // Save data to file
                writeln!(
                    file,
                    "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                    timestamp,
                    data.accelerometer[0],
                    data.accelerometer[1],
                    data.accelerometer[2],
                    data.gyroscope[0],
                    data.gyroscope[1],
                    data.gyroscope[2],
                    data.angle[0],
                    data.angle[1],
                    data.angle[2],
                    quat_x,
                    quat_y,
                    quat_z,
                    quat_w,
                    proj_grav_x,
                    proj_grav_y,
                    proj_grav_z
                )?;
            }
            Err(e) => eprintln!("Error reading from IMU: {}", e),
        }

        thread::sleep(Duration::from_millis(10));
    }
}
