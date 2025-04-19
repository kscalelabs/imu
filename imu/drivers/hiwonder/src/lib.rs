pub use imu_traits::{ImuData, ImuError, ImuFrequency, ImuReader, Quaternion, Vector3};
use serialport;
use std::io::{self, Read};
use std::sync::{mpsc, Arc, RwLock};
use std::thread;
use std::time::Duration;

#[derive(Debug, PartialEq)]
enum FrameState {
    Idle,
    Acc,
    Gyro,
    Angle,
    Quaternion,
    Mag,
}

pub trait FrequencyToByte {
    fn to_byte(&self) -> u8;
}

impl FrequencyToByte for ImuFrequency {
    fn to_byte(&self) -> u8 {
        match self {
            ImuFrequency::Hz0_2 => 0x01,
            ImuFrequency::Hz0_5 => 0x02,
            ImuFrequency::Hz1 => 0x03,
            ImuFrequency::Hz2 => 0x04,
            ImuFrequency::Hz5 => 0x05,
            ImuFrequency::Hz10 => 0x06,
            ImuFrequency::Hz20 => 0x07,
            ImuFrequency::Hz50 => 0x08,
            ImuFrequency::Hz100 => 0x09,
            ImuFrequency::Hz200 => 0x0B,
            ImuFrequency::Single => 0x0C,
            ImuFrequency::None => 0x0D,
        }
    }
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
    mag_data: [u8; 8],
    acc: [f32; 3],
    gyro: [f32; 3],
    angle: [f32; 3],
    quaternion: [f32; 4],
    mag: [f32; 3],
    temp: f32,
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
            mag_data: [0u8; 8],
            acc: [0.0; 3],
            gyro: [0.0; 3],
            angle: [0.0; 3],
            quaternion: [0.0; 4],
            mag: [0.0; 3],
            temp: 0.0,
        };

        imu.initialize()?;
        Ok(imu)
    }

    fn initialize(&mut self) -> Result<(), ImuError> {
        // * Set IMU Parameters to Read
        // Commands as vectors
        let unlock_cmd = vec![0xFF, 0xAA, 0x69, 0x88, 0xB5];
        // Enable Acceleration (0x02), Gyro (0x04), Angle (0x08), Mag (0x10), and Quaternion (0x0200)
        // Low byte: 0x02 | 0x04 | 0x08 | 0x10 = 0x1E
        // High byte: 0x02
        // let config_cmd = vec![0xFF, 0xAA, 0x02, 0x1E, 0x02];
        let config_cmd = vec![0xFF, 0xAA, 0x02, 0xFF, 0x07]; // 0x07FF = all bits set
        let save_cmd = vec![0xFF, 0xAA, 0x00, 0x00, 0x00];

        // Alternative:
        // let mut packet = Vec::with_capacity(5 + data.len());
        // packet.push(0x55); // many of these...
        // self.port.write_all(&packet)

        // Send commands in sequence.
        self.write_command(&unlock_cmd)?;
        self.write_command(&config_cmd)?;
        self.write_command(&save_cmd)?;

        // Set IMU frequency to a reasonable default.
        self.set_frequency(ImuFrequency::Hz100)?;
        Ok(())
    }

    fn write_command(&mut self, command: &[u8]) -> Result<(), ImuError> {
        self.port.write_all(command).map_err(ImuError::from)?;
        // 200 hz -> 5ms
        std::thread::sleep(Duration::from_millis(30));
        Ok(())
    }

    pub fn set_frequency(&mut self, frequency: ImuFrequency) -> Result<(), ImuError> {
        let freq_cmd = vec![0xFF, 0xAA, 0x03, frequency.to_byte(), 0x00];
        self.write_command(&freq_cmd)?;
        Ok(())
    }

    pub fn read_data(
        &mut self,
    ) -> io::Result<Option<([f32; 3], [f32; 3], [f32; 3], [f32; 4], [f32; 3], f32)>> {
        let mut buffer = vec![0; 1024];
        match self.port.read(&mut buffer) {
            Ok(bytes_read) if bytes_read > 0 => {
                self.process_data(&buffer[..bytes_read]);
                // Only return data when we have a complete angle reading
                if self.frame_state == FrameState::Idle {
                    Ok(Some((
                        self.acc,
                        self.gyro,
                        self.angle,
                        self.quaternion,
                        self.mag,
                        self.temp,
                    )))
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
                            0x54 => {
                                self.frame_state = FrameState::Mag;
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
                            self.temp = Self::get_temperature(&self.acc_data);
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
                FrameState::Mag => {
                    if self.byte_num < 10 {
                        self.mag_data[self.byte_num - 2] = data;
                        self.checksum = self.checksum.wrapping_add(data);
                        self.byte_num += 1;
                    } else {
                        if data == (self.checksum & 0xFF) {
                            self.mag = Self::get_mag(&self.mag_data);
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
        let k_acc = 16.0 * 9.80665;
        let acc_x = i16::from(datahex[1]) << 8 | i16::from(datahex[0]);
        let acc_y = i16::from(datahex[3]) << 8 | i16::from(datahex[2]);
        let acc_z = i16::from(datahex[5]) << 8 | i16::from(datahex[4]);

        [
            acc_x as f32 / 32768.0 * k_acc,
            acc_y as f32 / 32768.0 * k_acc,
            acc_z as f32 / 32768.0 * k_acc,
        ]
    }

    fn get_gyro(datahex: &[u8; 8]) -> [f32; 3] {
        let k_gyro = 2000.0 * std::f32::consts::PI / 180.0;
        let gyro_x = i16::from(datahex[1]) << 8 | i16::from(datahex[0]);
        let gyro_y = i16::from(datahex[3]) << 8 | i16::from(datahex[2]);
        let gyro_z = i16::from(datahex[5]) << 8 | i16::from(datahex[4]);

        [
            gyro_x as f32 / 32768.0 * k_gyro,
            gyro_y as f32 / 32768.0 * k_gyro,
            gyro_z as f32 / 32768.0 * k_gyro,
        ]
    }

    fn get_mag(datahex: &[u8; 8]) -> [f32; 3] {
        let mag_x = i16::from_le_bytes([datahex[0], datahex[1]]);
        let mag_y = i16::from_le_bytes([datahex[2], datahex[3]]);
        let mag_z = i16::from_le_bytes([datahex[4], datahex[5]]);
        [mag_x as f32, mag_y as f32, mag_z as f32]
    }

    fn get_temperature(datahex: &[u8; 8]) -> f32 {
        let temp_raw = i16::from_le_bytes([datahex[6], datahex[7]]);
        temp_raw as f32 / 100.0
    }

    fn get_angle(datahex: &[u8; 8]) -> [f32; 3] {
        let k_angle = std::f32::consts::PI;
        let angle_x = i16::from(datahex[1]) << 8 | i16::from(datahex[0]);
        let angle_y = i16::from(datahex[3]) << 8 | i16::from(datahex[2]);
        let angle_z = i16::from(datahex[5]) << 8 | i16::from(datahex[4]);

        [
            angle_x as f32 / 32768.0 * k_angle,
            angle_y as f32 / 32768.0 * k_angle,
            angle_z as f32 / 32768.0 * k_angle,
        ]
    }

    fn get_quaternion(datahex: &[u8; 8]) -> [f32; 4] {
        let quaternion_w = i16::from(datahex[1]) << 8 | i16::from(datahex[0]);
        let quaternion_x = i16::from(datahex[3]) << 8 | i16::from(datahex[2]);
        let quaternion_y = i16::from(datahex[5]) << 8 | i16::from(datahex[4]);
        let quaternion_z = i16::from(datahex[7]) << 8 | i16::from(datahex[6]);

        [
            quaternion_x as f32 / 32768.0,
            quaternion_y as f32 / 32768.0,
            quaternion_z as f32 / 32768.0,
            quaternion_w as f32 / 32768.0,
        ]
    }
}

pub struct HiwonderReader {
    data: Arc<RwLock<ImuData>>,
    command_tx: mpsc::Sender<ImuCommand>,
    running: Arc<RwLock<bool>>,
}

#[derive(Debug)]
pub enum ImuCommand {
    Reset,
    Stop,
    SetFrequency(ImuFrequency),
}

impl HiwonderReader {
    pub fn new(interface: &str, baud_rate: u32) -> Result<Self, ImuError> {
        let data = Arc::new(RwLock::new(ImuData::default()));
        let running = Arc::new(RwLock::new(true));
        let (command_tx, command_rx) = mpsc::channel();

        let reader = HiwonderReader {
            data: Arc::clone(&data),
            command_tx,
            running: Arc::clone(&running),
        };

        reader.start_reading_thread(interface, baud_rate, command_rx)?;

        Ok(reader)
    }

    fn start_reading_thread(
        &self,
        interface: &str,
        baud_rate: u32,
        command_rx: mpsc::Receiver<ImuCommand>,
    ) -> Result<(), ImuError> {
        let data = Arc::clone(&self.data);
        let running = Arc::clone(&self.running);
        let interface = interface.to_string();

        let (tx, rx) = mpsc::channel();

        thread::spawn(move || {
            // Initialize IMU inside the thread and send result back
            let init_result = IMU::new(&interface, baud_rate);
            if let Err(e) = init_result {
                let _ = tx.send(Err(e));
                return;
            }
            let mut imu = init_result.unwrap();
            let _ = tx.send(Ok(()));

            while let Ok(guard) = running.read() {
                if !*guard {
                    break;
                }

                // Check for any pending commands
                if let Ok(command) = command_rx.try_recv() {
                    match command {
                        ImuCommand::Reset => {
                            if let Err(e) = imu.initialize() {
                                eprintln!("Failed to reset IMU: {}", e);
                            }
                        }
                        ImuCommand::Stop => {
                            if let Ok(mut guard) = running.write() {
                                *guard = false;
                            }
                            break;
                        }
                        ImuCommand::SetFrequency(frequency) => {
                            if let Err(e) = imu.set_frequency(frequency) {
                                eprintln!("Failed to set frequency: {}", e);
                            }
                        }
                    }
                }

                // Read IMU data
                match imu.read_data() {
                    Ok(Some((acc, gyro, angle, quat, mag, temp))) => {
                        if let Ok(mut imu_data) = data.write() {
                            imu_data.accelerometer = Some(Vector3 {
                                x: acc[0],
                                y: acc[1],
                                z: acc[2],
                            });
                            imu_data.gyroscope = Some(Vector3 {
                                x: gyro[0],
                                y: gyro[1],
                                z: gyro[2],
                            });
                            imu_data.euler = Some(Vector3 {
                                x: angle[0],
                                y: angle[1],
                                z: angle[2],
                            });
                            imu_data.quaternion = Some(Quaternion {
                                w: quat[3],
                                x: quat[0],
                                y: quat[1],
                                z: quat[2],
                            });
                            imu_data.magnetometer = Some(Vector3 {
                                x: mag[0],
                                y: mag[1],
                                z: mag[2],
                            });
                            imu_data.temperature = Some(temp);
                        }
                    }
                    Ok(None) => (), // No complete data available yet
                    Err(e) => eprintln!("Error reading from IMU: {}", e),
                }

                // Sleep for a short duration to prevent busy waiting
                // Max frequency is 200hz, so 5ms is the max delay
                thread::sleep(Duration::from_millis(5));
            }
        });

        // Wait for initialization result before returning
        rx.recv()
            .map_err(|_| {
                ImuError::InvalidPacket("Failed to receive initialization result".to_string())
            })?
            .map_err(|e| e)
    }

    pub fn reset(&self) -> Result<(), ImuError> {
        self.command_tx.send(ImuCommand::Reset)?;
        Ok(())
    }

    pub fn set_frequency(&self, frequency: ImuFrequency) -> Result<(), ImuError> {
        self.command_tx.send(ImuCommand::SetFrequency(frequency))?;
        Ok(())
    }
}

impl ImuReader for HiwonderReader {
    fn stop(&self) -> Result<(), ImuError> {
        self.command_tx.send(ImuCommand::Stop)?;
        Ok(())
    }

    fn get_data(&self) -> Result<ImuData, ImuError> {
        self.data
            .read()
            .map(|data| data.clone())
            .map_err(|_| ImuError::ReadError("Lock error".to_string()))
    }
}

impl Drop for HiwonderReader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
