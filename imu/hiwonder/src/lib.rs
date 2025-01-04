use serialport;
use std::io::{self, Read};
use std::time::Duration;

#[derive(Debug)]
pub enum ImuError {
    SerialError(serialport::Error),
    WriteError(std::io::Error),
    ReadError(std::io::Error),
    InvalidPacket,
}

impl From<std::io::Error> for ImuError {
    fn from(error: std::io::Error) -> Self {
        ImuError::WriteError(error)
    }
}

impl From<serialport::Error> for ImuError {
    fn from(error: serialport::Error) -> Self {
        ImuError::SerialError(error)
    }

}

impl std::fmt::Display for ImuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ImuError::ReadError(e) => write!(f, "Read error: {}", e),
            ImuError::WriteError(e) => write!(f, "Write error: {}", e),
            ImuError::SerialError(e) => write!(f, "Serial error: {}", e),
            ImuError::InvalidPacket => write!(f, "Invalid packet"),
        }
    }
}

impl std::error::Error for ImuError {}

#[derive(Debug, PartialEq)]
enum FrameState {
    Idle,
    Acc,
    Gyro,
    Angle,
    Quaternion,
}

pub struct IMU {
    port: Box<dyn serialport::SerialPort>,
    frame_state: FrameState,
    byte_num: usize,
    checksum: u8,
    acc_data: [u8; 8],
    gyro_data: [u8; 8],
    angle_data: [u8; 8],
    quaternion_data: [u8; 8],
    acc: [f32; 3],
    gyro: [f32; 3],
    angle: [f32; 3],
    quaternion: [f32; 4],
}


impl IMU {
    pub fn new(interface: &str, baud_rate: u32) -> Result<Self, ImuError> {
        let port = serialport::new(interface, baud_rate)
            .timeout(Duration::from_millis(500))
            .open()?;

        let mut imu = IMU {
            port: port,
            frame_state: FrameState::Idle,
            byte_num: 0,
            checksum: 0,
            acc_data: [0u8; 8],
            gyro_data: [0u8; 8],
            angle_data: [0u8; 8],
            quaternion_data: [0u8; 8],
            acc: [0.0; 3],
            gyro: [0.0; 3],
            angle: [0.0; 3],
            quaternion: [0.0; 4],
        };

        imu.initialize()?;
        Ok(imu)
    }

    fn initialize(&mut self) -> Result<(), ImuError> {

        // * Set IMU Parameters to Read
        // Commands as vectors
        let unlock_cmd = vec![0xFF, 0xAA, 0x69, 0x88, 0xB5];
        let config_cmd = vec![0xFF, 0xAA, 0x02, 0x0E, 0x02];
        let save_cmd = vec![0xFF, 0xAA, 0x00, 0x00, 0x00];

        // Alternative:
        // let mut packet = Vec::with_capacity(5 + data.len());
        // packet.push(0x55); // many of these...
        // self.port.write_all(&packet)

        // Send commands in sequence
        self.write_command(&unlock_cmd)?;
        self.write_command(&config_cmd)?;
        self.write_command(&save_cmd)?;


        // * Set IMU Freq 
        // RATE (0x03) Hz: 0x01=0.2, 0x02=0.5, 0x03=1, 0x04=2, 0x05=5, 0x06=10, 
        //0x07=20, 0x08=50, 0x09=100, 0x0B=200hz, 0x0C=Single, 0x0D=None
        let freq_cmd = vec![0xFF, 0xAA, 0x03, 0x04, 0x00]; // 0x0B 200hz
        self.write_command(&freq_cmd)?;
        Ok(())
    }

    fn write_command(&mut self, command: &[u8]) -> Result<(), ImuError> {
        self.port.write_all(command).map_err(ImuError::WriteError)?;
        // 200 hz -> 5ms
        std::thread::sleep(Duration::from_millis(30));
        Ok(())
    }

    pub fn read_data(&mut self) -> io::Result<Option<([f32; 3], [f32; 3], [f32; 3], [f32; 4])>> {
        let mut buffer = vec![0; 1024];
        match self.port.read(&mut buffer) {
            Ok(bytes_read) if bytes_read > 0 => {
                self.process_data(&buffer[..bytes_read]);
                // Only return data when we have a complete angle reading
                if self.frame_state == FrameState::Idle {
                    Ok(Some((self.acc, self.gyro, self.angle, self.quaternion)))
                } else {
                    Ok(None)
                }
            }
            Ok(_) => Ok(None),
            Err(e) => Err(e),
        }
    }

    pub fn process_data(&mut self, input_data: &[u8]) {
        for &data in input_data {
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
                            0x59 => {
                                // println!("frame_state: {:?}", self.frame_state);
                                self.frame_state = FrameState::Quaternion;
                                self.byte_num = 2;
                            }
                            _ => {
                                self.reset();
                            }
                        }
                    }
                }
                // 11 bytes per packet for all, including the SOF. 
                FrameState::Acc => {
                    if self.byte_num < 10 {
                        self.acc_data[self.byte_num - 2] = data;
                        self.checksum = self.checksum.wrapping_add(data);
                        self.byte_num += 1;
                    } else {
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
                        }
                        self.reset();
                    }
                }
                FrameState::Quaternion => {
                    if self.byte_num < 10 {
                        self.quaternion_data[self.byte_num - 2] = data;
                        self.checksum = self.checksum.wrapping_add(data);
                        self.byte_num += 1;
                    } else {
                        if data == (self.checksum & 0xFF) {
                            self.quaternion = Self::get_quaternion(&self.quaternion_data);
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
        let k_acc = 16.0;
        let acc_x = ((u16::from(datahex[1]) << 8) | u16::from(datahex[0])) as f32 / 32768.0 * k_acc;
        let acc_y = ((u16::from(datahex[3]) << 8) | u16::from(datahex[2])) as f32 / 32768.0 * k_acc;
        let acc_z = ((u16::from(datahex[5]) << 8) | u16::from(datahex[4])) as f32 / 32768.0 * k_acc;

        [
            if acc_x >= k_acc {
                acc_x - 2.0 * k_acc
            } else {
                acc_x
            },
            if acc_y >= k_acc {
                acc_y - 2.0 * k_acc
            } else {
                acc_y
            },
            if acc_z >= k_acc {
                acc_z - 2.0 * k_acc
            } else {
                acc_z
            },
        ]
    }

    fn get_gyro(datahex: &[u8; 8]) -> [f32; 3] {
        let k_gyro = 2000.0;
        let gyro_x =
            ((u16::from(datahex[1]) << 8) | u16::from(datahex[0])) as f32 / 32768.0 * k_gyro;
        let gyro_y =
            ((u16::from(datahex[3]) << 8) | u16::from(datahex[2])) as f32 / 32768.0 * k_gyro;
        let gyro_z =
            ((u16::from(datahex[5]) << 8) | u16::from(datahex[4])) as f32 / 32768.0 * k_gyro;

        [
            if gyro_x >= k_gyro {
                gyro_x - 2.0 * k_gyro
            } else {
                gyro_x
            },
            if gyro_y >= k_gyro {
                gyro_y - 2.0 * k_gyro
            } else {
                gyro_y
            },
            if gyro_z >= k_gyro {
                gyro_z - 2.0 * k_gyro
            } else {
                gyro_z
            },
        ]
    }

    fn get_angle(datahex: &[u8; 8]) -> [f32; 3] {
        let k_angle = 180.0;
        let angle_x =
            ((u16::from(datahex[1]) << 8) | u16::from(datahex[0])) as f32 / 32768.0 * k_angle;
        let angle_y =
            ((u16::from(datahex[3]) << 8) | u16::from(datahex[2])) as f32 / 32768.0 * k_angle;
        let angle_z =
            ((u16::from(datahex[5]) << 8) | u16::from(datahex[4])) as f32 / 32768.0 * k_angle;

        [
            if angle_x >= k_angle {
                angle_x - 2.0 * k_angle
            } else {
                angle_x
            },
            if angle_y >= k_angle {
                angle_y - 2.0 * k_angle
            } else {
                angle_y
            },
            if angle_z >= k_angle {
                angle_z - 2.0 * k_angle
            } else {
                angle_z
            },
        ]
    }

    fn get_quaternion(datahex: &[u8; 8]) -> [f32; 4] {
        let quaternion_x = ((u16::from(datahex[1]) << 8) | u16::from(datahex[0])) as f32 / 32768.0;
        let quaternion_y = ((u16::from(datahex[3]) << 8) | u16::from(datahex[2])) as f32 / 32768.0;
        let quaternion_z = ((u16::from(datahex[5]) << 8) | u16::from(datahex[4])) as f32 / 32768.0;
        let quaternion_w = ((u16::from(datahex[7]) << 8) | u16::from(datahex[6])) as f32 / 32768.0;
        [quaternion_x, quaternion_y, quaternion_z, quaternion_w]
    }
}
