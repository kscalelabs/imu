use hiwonder::IMU;
use std::time::Duration;
use std::io::{self, Read};

//* run by  `cargo run --bin log` */

fn main() -> io::Result<()> {    
    let port_name = "/dev/ttyUSB0";
    let baud_rate = 9600;

    let mut port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(500)) // 0.5 second timeout
        .open()
        .expect("Failed to open serial port");


    let mut imu = IMU::new();
    let mut buffer: Vec<u8> = vec![0; 1024];

    loop {
        match port.read(&mut buffer) {
            Ok(bytes_read) => {
                if bytes_read > 0 {
                    imu.process_data(&buffer[..bytes_read]);
                }
            }
            Err(e) => {
                eprintln!("Error reading from serial port: {}", e);
            }
        }
    }
}