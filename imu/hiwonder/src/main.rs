use serialport::SerialPort;
use std::io::Read;
use std::time::Duration;

fn main() {
    let port_name = "/dev/ttyUSB0";
    let baud_rate = 9600;

    // Attempt to open the serial port with the specified settings
    let mut port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(500)) // 0.5 second timeout
        .open()
        .expect("Failed to open serial port");

    // println!("Serial is Opened: {}", port.is_open());

    let mut buffer: [u8; 33] = [0; 33];

    loop {
        match port.read(&mut buffer) {
            Ok(bytes_read) => {
                if bytes_read > 0 {
                    let hex_string = buffer[..bytes_read]
                        .iter()
                        .map(|byte| format!("{:02X}", byte))
                        .collect::<Vec<String>>()
                        .join(" ");
                    println!("{}", hex_string);
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                // Timeout reached, continue reading
                continue;
            }
            Err(e) => {
                eprintln!("Error reading from serial port: {}", e);
                break;
            }
        }
    }
}