use std::io::{self, Read};
use std::time::Duration;


#[derive(Debug)]
enum FrameState {
    Idle,
    Acc,
    Gyro,
    Angle,
}

struct IMU {
    frame_state: FrameState,
    byte_num: usize,
    checksum: u8,
    acc_data: [u8; 8],
    gyro_data: [u8; 8],
    angle_data: [u8; 8],
    acc: [f32; 3],
    gyro: [f32; 3],
    angle: [f32; 3],
}

impl IMU {
    fn new() -> Self {
        IMU {
            frame_state: FrameState::Idle,
            byte_num: 0,
            checksum: 0,
            acc_data: [0u8; 8],
            gyro_data: [0u8; 8],
            angle_data: [0u8; 8],
            acc: [0.0; 3],
            gyro: [0.0; 3],
            angle: [0.0; 3],
        }
    }

    fn process_data(&mut self, input_data: &[u8]) {
        for &data in input_data {
            // println!("frame: {:?}", self.frame_state);
            // println!("byte_num: {}", self.byte_num);
            // println!("data: 0x{:02x}", data);
            // println!("angle: {:10.3} {:10.3} {:10.3}", self.angle[0], self.angle[1], self.angle[2]);
            match self.frame_state {
                FrameState::Idle => {
                    if data == 0x55 && self.byte_num == 0 {
                        self.checksum = data;
                        self.byte_num = 1;
                        continue;
                    } else if self.byte_num == 1 {
                        self.checksum = self.checksum.wrapping_add(data);
                        match data {
                            0x51 => {
                                self.frame_state = FrameState::Acc;
                                self.byte_num = 2;
                            }
                            0x52 => {
                                self.frame_state = FrameState::Gyro;
                                self.byte_num = 2;
                            }
                            0x53 => {
                                self.frame_state = FrameState::Angle;
                                self.byte_num = 2;
                            }
                            _ => {
                                self.reset();
                            }
                        }
                    }
                }
                FrameState::Acc => {
                    // println!("Acc state: byte_num={}, data=0x{:02x}, checksum=0x{:02x}", self.byte_num, data, self.checksum);
                    if self.byte_num < 10 {
                        self.acc_data[self.byte_num - 2] = data;
                        self.checksum = self.checksum.wrapping_add(data);
                        self.byte_num += 1;
                    } else {
                        // println!("Acc checksum compare: data=0x{:02x}, calculated=0x{:02x}", data, self.checksum & 0xFF);
                        if data == (self.checksum & 0xFF) {
                            self.acc = Self::get_acc(&self.acc_data);
                        }
                        self.reset();
                    }
                }
                FrameState::Gyro => {
                    if self.byte_num < 10 {
                        self.gyro_data[self.byte_num - 2] = data;
                        self.checksum = self.checksum.wrapping_add(data);
                        self.byte_num += 1;
                    } else {
                        if data == (self.checksum & 0xFF) {
                            self.gyro = Self::get_gyro(&self.gyro_data);
                        }
                        self.reset();
                    }
                }
                FrameState::Angle => {
                    if self.byte_num < 10 {
                        self.angle_data[self.byte_num - 2] = data;
                        self.checksum = self.checksum.wrapping_add(data);
                        self.byte_num += 1;
                    } else {
                        if data == (self.checksum & 0xFF) {
                            self.angle = Self::get_angle(&self.angle_data);
                            self.print_results();
                        }
                        self.reset();
                    }
                }
            }
        }
    }

    fn reset(&mut self) {
        self.frame_state = FrameState::Idle;
        self.byte_num = 0;
        self.checksum = 0;
    }

    fn get_acc(datahex: &[u8; 8]) -> [f32; 3] {
        println!("Called Get Acc: {:02x?}", datahex);
        let k_acc = 16.0;
        let acc_x = ((u16::from(datahex[1]) << 8) | u16::from(datahex[0])) as f32 / 32768.0 * k_acc;
        let acc_y = ((u16::from(datahex[3]) << 8) | u16::from(datahex[2])) as f32 / 32768.0 * k_acc;
        let acc_z = ((u16::from(datahex[5]) << 8) | u16::from(datahex[4])) as f32 / 32768.0 * k_acc;

        [
            if acc_x >= k_acc { acc_x - 2.0 * k_acc } else { acc_x },
            if acc_y >= k_acc { acc_y - 2.0 * k_acc } else { acc_y },
            if acc_z >= k_acc { acc_z - 2.0 * k_acc } else { acc_z },
        ]
    }

    fn get_gyro(datahex: &[u8; 8]) -> [f32; 3] {
        let k_gyro = 2000.0;
        let gyro_x = ((u16::from(datahex[1]) << 8) | u16::from(datahex[0])) as f32 / 32768.0 * k_gyro;
        let gyro_y = ((u16::from(datahex[3]) << 8) | u16::from(datahex[2])) as f32 / 32768.0 * k_gyro;
        let gyro_z = ((u16::from(datahex[5]) << 8) | u16::from(datahex[4])) as f32 / 32768.0 * k_gyro;

        [
            if gyro_x >= k_gyro { gyro_x - 2.0 * k_gyro } else { gyro_x },
            if gyro_y >= k_gyro { gyro_y - 2.0 * k_gyro } else { gyro_y },
            if gyro_z >= k_gyro { gyro_z - 2.0 * k_gyro } else { gyro_z },
        ]
    }

    fn get_angle(datahex: &[u8; 8]) -> [f32; 3] {
        let k_angle = 180.0;
        let angle_x = ((u16::from(datahex[1]) << 8) | u16::from(datahex[0])) as f32 / 32768.0 * k_angle;
        let angle_y = ((u16::from(datahex[3]) << 8) | u16::from(datahex[2])) as f32 / 32768.0 * k_angle;
        let angle_z = ((u16::from(datahex[5]) << 8) | u16::from(datahex[4])) as f32 / 32768.0 * k_angle;

        [
            if angle_x >= k_angle { angle_x - 2.0 * k_angle } else { angle_x },
            if angle_y >= k_angle { angle_y - 2.0 * k_angle } else { angle_y },
            if angle_z >= k_angle { angle_z - 2.0 * k_angle } else { angle_z },
        ]
    }

    fn print_results(&self) {
        println!(
            "acc: {:10.3} {:10.3} {:10.3}\ngyro: {:10.3} {:10.3} {:10.3}\nangle: {:10.3} {:10.3} {:10.3}",
            self.acc[0], self.acc[1], self.acc[2],
            self.gyro[0], self.gyro[1], self.gyro[2],
            self.angle[0], self.angle[1], self.angle[2]
        );
    }
}

fn main() -> io::Result<()> {
    // You can also take port and baud rate from command line arguments
    let port = "/dev/ttyUSB0";
    let baud_rate = 9600;

    let port_name = "/dev/ttyUSB0";

    let mut port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(500)) // 0.5 second timeout
        .open()
        .expect("Failed to open serial port");


    println!("Serial port {} opened.", port.name().unwrap());

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