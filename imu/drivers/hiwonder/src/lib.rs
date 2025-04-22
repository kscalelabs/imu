pub mod frame;
pub mod register;
pub use frame::*;
pub use imu_traits::{ImuData, ImuError, ImuFrequency, ImuReader, Quaternion, Vector3};
pub use register::*;
use std::sync::{mpsc, Arc, RwLock};
use std::thread;
use std::time::Duration;

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
    frame_parser: FrameParser,
}

impl IMU {
    pub fn new(interface: &str, baud_rate: u32) -> Result<Self, ImuError> {
        let port = serialport::new(interface, baud_rate)
            .timeout(Duration::from_millis(500))
            .open()?;

        let mut imu = IMU {
            port,
            frame_parser: FrameParser::new(Some(512)),
        };

        imu.initialize()?;
        Ok(imu)
    }

    fn initialize(&mut self) -> Result<(), ImuError> {
        let enabled_outputs = Output::ACC | Output::GYRO | Output::ANGLE | Output::QUATERNION;

        // Send commands in sequence.
        self.write_command(&UnlockCommand::new())?; // Unlock
        self.write_command(&FusionAlgorithmCommand::new(FusionAlgorithm::SixAxis))?; // Axis6
        self.write_command(&EnableOutputCommand::new(enabled_outputs))?; // Enable
        self.write_command(&SaveCommand::new())?; // Save

        // Set IMU frequency to a reasonable default.
        self.write_command(&SetFrequencyCommand::new(ImuFrequency::Hz100))?;

        Ok(())
    }

    fn write_command(&mut self, command: &dyn Bytable) -> Result<(), ImuError> {
        self.port
            .write_all(&command.to_bytes())
            .map_err(ImuError::from)?;
        // 200 hz -> 5ms
        std::thread::sleep(Duration::from_millis(30));
        Ok(())
    }

    pub fn set_frequency(&mut self, frequency: ImuFrequency) -> Result<(), ImuError> {
        self.write_command(&UnlockCommand::new())?;
        self.write_command(&SetFrequencyCommand::new(frequency))?;
        self.write_command(&SaveCommand::new())?;
        Ok(())
    }

    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<(), ImuError> {
        self.write_command(&UnlockCommand::new())?;
        self.write_command(&SetBaudRateCommand::new(BaudRate::try_from(baud_rate)?))?;
        self.write_command(&SaveCommand::new())?;
        self.port.set_baud_rate(baud_rate)?;
        Ok(())
    }

    pub fn get_frames(&mut self) -> Result<Vec<ReadFrame>, ImuError> {
        let mut buffer = [0u8; 1024];
        match self.port.read(&mut buffer) {
            Ok(n) => {
                if n > 0 {
                    Ok(self.frame_parser.parse(&buffer[0..n])?)
                } else {
                    Ok(vec![])
                }
            }
            Err(e) => Err(ImuError::ReadError(format!("Failed to read data: {}", e))),
        }
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
    SetBaudRate(u32),
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

            let mut imu = init_result.unwrap(); // This is safe because we have already checked for errors
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
                        ImuCommand::SetBaudRate(baud_rate) => {
                            if let Err(e) = imu.set_baud_rate(baud_rate) {
                                eprintln!("Failed to set baud rate: {}", e);
                            }
                        }
                    }
                }

                // Read IMU data
                match imu.get_frames() {
                    Ok(frames) => {
                        for frame in frames {
                            if let Ok(mut imu_data) = data.write() {
                                match frame {
                                    ReadFrame::Acceleration { x, y, z, temp: _ } => {
                                        imu_data.accelerometer = Some(Vector3 { x, y, z });
                                    }
                                    ReadFrame::Gyro {
                                        x,
                                        y,
                                        z,
                                        voltage: _,
                                    } => {
                                        imu_data.gyroscope = Some(Vector3 { x, y, z });
                                    }
                                    ReadFrame::Angle {
                                        roll,
                                        pitch,
                                        yaw,
                                        version: _,
                                    } => {
                                        imu_data.euler = Some(Vector3 {
                                            x: roll,
                                            y: pitch,
                                            z: yaw,
                                        });
                                    }
                                    ReadFrame::Magnetometer { x, y, z, temp: _ } => {
                                        imu_data.magnetometer = Some(Vector3 { x, y, z });
                                    }
                                    ReadFrame::Quaternion { w, x, y, z } => {
                                        imu_data.quaternion = Some(Quaternion { w, x, y, z });
                                    }
                                    _ => (),
                                }
                            } else {
                                eprintln!("Failed to write to IMU data");
                            }
                        }
                    }
                    Err(e) => eprintln!("Error reading from IMU: {}", e),
                }

                // Sleep for a short duration to prevent busy waiting
                // Max frequency is 200hz (5ms)
                thread::sleep(Duration::from_millis(4));
            }
        });

        // Wait for initialization result before returning
        rx.recv().map_err(|_| {
            ImuError::InvalidPacket("Failed to receive initialization result".to_string())
        })?
    }

    pub fn reset(&self) -> Result<(), ImuError> {
        self.command_tx.send(ImuCommand::Reset)?;
        Ok(())
    }

    pub fn set_frequency(&self, frequency: ImuFrequency) -> Result<(), ImuError> {
        self.command_tx.send(ImuCommand::SetFrequency(frequency))?;
        Ok(())
    }

    pub fn set_baud_rate(&self, baud_rate: u32) -> Result<(), ImuError> {
        self.command_tx.send(ImuCommand::SetBaudRate(baud_rate))?;
        Ok(())
    }
}

impl ImuReader for HiwonderReader {
    fn stop(&self) -> Result<(), ImuError> {
        self.command_tx.send(ImuCommand::Stop)?;
        Ok(())
    }

    fn get_data(&self) -> Result<ImuData, ImuError> {
        // Get the data
        let result = self
            .data
            .read()
            .map(|data| *data)
            .map_err(|_| ImuError::ReadError("Lock error".to_string()));

        result
    }
}

impl Drop for HiwonderReader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
