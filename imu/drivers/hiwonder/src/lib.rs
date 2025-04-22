pub mod frame;

pub use imu_traits::{ImuData, ImuError, ImuFrequency, ImuReader, Quaternion, Vector3};
use serialport;
use std::io::{self, Read};
use std::sync::{mpsc, Arc, RwLock};
use std::thread;
use std::time::Duration;
pub use frame::*;

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
            port: port,
            frame_parser: FrameParser::new(Some(512)),
        };

        imu.initialize()?;
        Ok(imu)
    }

    fn initialize(&mut self) -> Result<(), ImuError> {
        let low_byte = 0x02 | 0x04 | 0x08;
        let high_byte = 0x02;

        // Send commands in sequence.
        self.write_command(&vec![0xFF, 0xAA, 0x69, 0x88, 0xB5])?; // Unlock
        self.write_command(&vec![0xFF, 0xAA, 0x24, 0x01, 0x00])?; // Axis6
        self.write_command(&vec![0xFF, 0xAA, 0x02, low_byte, high_byte])?; // Enable
        self.write_command(&vec![0xFF, 0xAA, 0x00, 0x00, 0xFF])?; // Reboot

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

    pub fn read_data(&mut self) -> Result<Vec<ReadFrame>, ImuError> {
        let mut buffer = [0u8; 1024];
        match self.port.read(&mut buffer) {
            Ok(n) => {
                if n > 0 {
                    Ok(self.frame_parser.parse(&buffer[0..n])?)
                }else {
                    Ok(vec![])
                }
            }
            Err(e) => {
                return Err(ImuError::ReadError(format!("Failed to read data: {}", e)));
            }
        }
    }
}

pub struct HiwonderReader {
    data: Arc<RwLock<ImuData>>,
    command_tx: mpsc::Sender<ImuCommand>,
    running: Arc<RwLock<bool>>,
    data_read: Arc<RwLock<bool>>, // Track if data has been read
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
        let data_read = Arc::new(RwLock::new(true));
        let (command_tx, command_rx) = mpsc::channel();

        let reader = HiwonderReader {
            data: Arc::clone(&data),
            command_tx,
            running: Arc::clone(&running),
            data_read: Arc::clone(&data_read),
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
        let data_read = Arc::clone(&self.data_read);
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
                    Ok(frames) => {
                        for frame in frames {
                            match frame {
                                ReadFrame::Acceleration { x, y, z, temp: _ } => {
                                    data.write().unwrap().accelerometer = Some(Vector3 { x, y, z });
                                }
                                ReadFrame::Gyro { x, y, z, voltage: _ } => {
                                    data.write().unwrap().gyroscope = Some(Vector3 { x, y, z });
                                }
                                ReadFrame::Angle { roll, pitch, yaw, version: _ } => {
                                    data.write().unwrap().euler = Some(Vector3 { x: roll, y: pitch, z: yaw });
                                }
                                ReadFrame::Magnetometer { x, y, z, temp: _ } => {
                                    data.write().unwrap().magnetometer = Some(Vector3 { x, y, z });
                                }
                                ReadFrame::Quaternion { w, x, y, z } => {
                                    data.write().unwrap().quaternion = Some(Quaternion { w, x, y, z });
                                }
                                _ => (),
                            }
                        }
                    }
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
        // Check if data has been read
        if let Ok(read) = self.data_read.read() {
            if *read {
                return Err(ImuError::ReadError("No new data available".to_string()));
            }
        }

        // Get the data
        let result = self
            .data
            .read()
            .map(|data| data.clone())
            .map_err(|_| ImuError::ReadError("Lock error".to_string()));

        // Mark data as read if we successfully got it
        if result.is_ok() {
            if let Ok(mut read) = self.data_read.write() {
                *read = true;
            }
        }

        result
    }
}

impl Drop for HiwonderReader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
