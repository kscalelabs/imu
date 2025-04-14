use hiwonder::{ImuFrequency, HiwonderReader};
use std::io;
use std::thread;
use std::time::{Duration, SystemTime};
use std::fs::File;
use std::io::Write;

fn main() -> io::Result<()> {
    let reader =
        HiwonderReader::new("/dev/ttyUSB0", 9600).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    match reader.set_frequency(ImuFrequency::Hz200) {
        Ok(_) => println!("Set frequency to 200hz"),
        Err(e) => println!("Failed to set frequency: {}", e),
    }

    match reader.set_acc_filter(500) {
        Ok(_) => println!("Successfully set acc filter to 500"),
        Err(e) => println!("Failed to set acc filter: {}", e),
    }

    let mut file = File::create("imu_data.csv")?;
    writeln!(file, "timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z,quat_x,quat_y,quat_z,quat_w")?;

    let start_time = SystemTime::now();

    loop {
        match reader.get_data() {
            Ok(data) => {
                let timestamp = SystemTime::now()
                    .duration_since(start_time)
                    .unwrap()
                    .as_secs_f64();

                writeln!(
                    file,
                    "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
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
                    data.quaternion[0],
                    data.quaternion[1],
                    data.quaternion[2],
                    data.quaternion[3],
                )?;

                // println!(
                //     "acc:   x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                //      gyro:  x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                //      angle: x: {: >10.3} y: {: >10.3} z: {: >10.3}\n\
                //      quaternion: x: {: >10.3} y: {: >10.3} z: {: >10.3} w: {: >10.3}",
                //     data.accelerometer[0],
                //     data.accelerometer[1],
                //     data.accelerometer[2],
                //     data.gyroscope[0],
                //     data.gyroscope[1],
                //     data.gyroscope[2],
                //     data.angle[0],
                //     data.angle[1],
                //     data.angle[2],
                //     data.quaternion[0],
                //     data.quaternion[1],
                //     data.quaternion[2],
                //     data.quaternion[3],
                // );
            }
            Err(e) => eprintln!("Error reading from IMU: {}", e),
        }

        thread::sleep(Duration::from_millis(10));
    }
}
