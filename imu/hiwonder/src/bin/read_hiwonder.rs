use hiwonder::{ImuFrequency, IMU};
use std::io;

//* run by  `cargo run --bin log` */
#[derive(Debug)]
struct IMUData {
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    angle_x: f32,
    angle_y: f32,
    angle_z: f32,
    quaternion_x: f32,
    quaternion_y: f32,
    quaternion_z: f32,
    quaternion_w: f32,
}

impl From<([f32; 3], [f32; 3], [f32; 3], [f32; 4])> for IMUData {
    fn from((acc, gyro, angle, quaternion): ([f32; 3], [f32; 3], [f32; 3], [f32; 4])) -> Self {
        IMUData {
            acc_x: acc[0],
            acc_y: acc[1],
            acc_z: acc[2],
            gyro_x: gyro[0],
            gyro_y: gyro[1],
            gyro_z: gyro[2],
            angle_x: angle[0],
            angle_y: angle[1],
            angle_z: angle[2],
            quaternion_x: quaternion[0],
            quaternion_y: quaternion[1],
            quaternion_z: quaternion[2],
            quaternion_w: quaternion[3],
        }
    }
}

fn main() -> io::Result<()> {
    let mut imu =
        IMU::new("/dev/ttyUSB0", 9600).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    match imu.set_frequency(ImuFrequency::Hz20) {
        Ok(_) => println!("Set frequency to 200hz"),
        Err(e) => println!("Failed to set frequency: {}", e),
    }

    loop {
        match imu.read_data() {
            Ok(Some(data)) => {
                let data = IMUData::from(data);
                println!(
                    "acc:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     gyro:  x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     angle: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                     quaternion: x: {: >10.3} y: {: >10.3} z: {: >10.3} w: {: >10.3}",
                    data.acc_x,
                    data.acc_y,
                    data.acc_z,
                    data.gyro_x,
                    data.gyro_y,
                    data.gyro_z,
                    data.angle_x,
                    data.angle_y,
                    data.angle_z,
                    data.quaternion_x,
                    data.quaternion_y,
                    data.quaternion_z,
                    data.quaternion_w
                );
            }
            Ok(None) => (), // No complete data available yet
            Err(e) => eprintln!("Error reading from serial port: {}", e),
        }
    }
}
