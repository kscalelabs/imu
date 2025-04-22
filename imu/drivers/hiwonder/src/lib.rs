pub mod frame;
pub mod register;
pub use frame::*;
pub use imu_traits::{ImuData, ImuError, ImuFrequency, ImuReader, Quaternion, Vector3};
pub use register::*;
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::{Duration, Instant};
use tracing::{debug, error, info, warn};

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

#[derive(PartialEq, Eq, Debug)]
pub enum ImuMode {
    Read,
    Write,
}

pub struct HiwonderReader {
    data: Arc<RwLock<ImuData>>,
    mode: Arc<RwLock<ImuMode>>,
    port: Arc<Mutex<Box<dyn serialport::SerialPort>>>,
    running: Arc<RwLock<bool>>,
    frame_parser: Arc<Mutex<FrameParser>>,
    timeout: Duration,
}

#[derive(Debug)]
pub enum ImuCommand {
    Reset,
    Stop,
    SetFrequency(ImuFrequency),
    SetBaudRate(u32),
}

impl HiwonderReader {
    pub fn new(interface: &str, baud_rate: u32, timeout: Duration) -> Result<Self, ImuError> {
        let data = Arc::new(RwLock::new(ImuData::default()));
        let running = Arc::new(RwLock::new(true));

        let port = serialport::new(interface, baud_rate)
            .timeout(timeout)
            .open()?;

        let reader = HiwonderReader {
            data: Arc::clone(&data),
            mode: Arc::new(RwLock::new(ImuMode::Read)),
            port: Arc::new(Mutex::new(port)),
            running: Arc::clone(&running),
            frame_parser: Arc::new(Mutex::new(FrameParser::new(Some(512)))),
            timeout,
        };

        let enabled_outputs = Output::empty(); //Output::ACC | Output::GYRO | Output::ANGLE | Output::QUATERNION;

        reader.write_command(&UnlockCommand::new(), false, Duration::from_secs(10))?;
        reader.write_command(&EnableOutputCommand::new(enabled_outputs), false, Duration::from_secs(10))?;
        reader.write_command(&SaveCommand::new(), false, Duration::from_secs(10))?;
        reader.write_command(&SetFrequencyCommand::new(ImuFrequency::Hz200), true, Duration::from_secs(10))?;

        reader.start_reading_thread()?;

        Ok(reader)
    }

    fn read_frames(
        port_arc: &Arc<Mutex<Box<dyn serialport::SerialPort>>>,
        parser_arc: &Arc<Mutex<FrameParser>>,
    ) -> Result<Vec<ReadFrame>, ImuError> {
        let mut buffer = [0u8; 1024];

        let mut port_guard = port_arc.lock().map_err(|e| {
            ImuError::ReadError(format!("Failed to acquire lock on port: {}", e))
        })?;

        let mut parser_guard = parser_arc.lock().map_err(|e| {
            ImuError::ReadError(format!(
                "Failed to acquire lock on frame parser: {}",
                e
            ))
        })?;

        match port_guard.read(&mut buffer) {
            Ok(n) => {
                if n > 0 {
                    parser_guard.parse(&buffer[0..n])
                } else {
                    warn!("No data read from port...");
                    Ok(vec![])
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(vec![]),
            Err(e) => Err(ImuError::ReadError(format!(
                "Failed to read data from port: {}",
                e
            ))),
        }
    }

    fn set_data(imu_data: &mut ImuData, frame: &ReadFrame){
        match *frame {
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
    }

    fn start_reading_thread(
        &self,
    ) -> Result<(), ImuError> {
        let data = Arc::clone(&self.data);
        let running = Arc::clone(&self.running);
        let port = Arc::clone(&self.port);
        let frame_parser = Arc::clone(&self.frame_parser);
        let mode = Arc::clone(&self.mode);

        thread::spawn(move || {
            while let Ok(guard) = running.read(){
                if !*guard {
                    break;
                }

                if let Ok(mode) = mode.read() {
                    if *mode == ImuMode::Read {
                        match Self::read_frames(&port, &frame_parser) {
                            Ok(frames) => {
                                if frames.is_empty() {
                                    warn!("No frames read from port");
                                }
                                for frame in frames {
                                    if let Ok(mut imu_data) = data.write() {
                                        Self::set_data(&mut imu_data, &frame);
                                    } else {
                                        error!("Failed to write to IMU data");
                                    }
                                }
                            }
                            Err(e) => error!("Error reading/parsing frames in thread: {}", e),
                        }
                    }else{
                        debug!("IMU is in write mode");
                    }

                    thread::sleep(Duration::from_millis(4));
                }
            }
        });

        Ok(())
    }

    pub fn reset(&self) -> Result<(), ImuError> {
        if let Ok(mut running_guard) = self.running.write() {
            *running_guard = false;
            Ok(())
        } else {
            Err(ImuError::ReadError("Failed to acquire lock for stop".to_string()))
        }
    }

    pub fn write_command(&self, command: &dyn BytableRegistrable, verify: bool, timeout: Duration) -> Result<(), ImuError> {

        if let Ok(mut mode_guard) = self.mode.write() {
            *mode_guard = ImuMode::Write;
        }

        let mut parser_guard = self.frame_parser.lock().map_err(|e| {
            ImuError::ReadError(format!("Failed to acquire lock on frame parser: {}", e))
        })?;

        if let Ok(mut port_guard) = self.port.lock() {
            port_guard.write_all(&command.to_bytes())
                .map_err(ImuError::from)?;

            std::thread::sleep(Duration::from_millis(30));

            if verify {
                // Send generic read to read data at the set addr
                port_guard.write_all(&ReadAddressCommand::new(command.register()).to_bytes())
                    .map_err(ImuError::from)?;

                info!("Sent command {:?} to IMU", command.to_bytes().as_slice());
                info!("Sent read command {:?} to IMU", ReadAddressCommand::new(command.register()).to_bytes().as_slice());

                let mut buffer = [0u8; 1024];
                let start_time = Instant::now();
                while start_time.elapsed() < timeout {
                    if let Ok(n) = port_guard.read(&mut buffer) {
                        if n > 0 {
                            if let Ok(response) = parser_guard.parse(&buffer[0..n]) {
                                for frame in response {
                                    match frame {
                                        ReadFrame::GenericRead { data } => {
                                            if data == command.to_bytes().as_slice() {
                                                info!("Command {:?} sent and verified", command.to_bytes().as_slice());
                                                return Ok(());
                                            }
                                        }
                                        _ => debug!("Received unexpected frame: {:?}", frame),
                                    }
                                }
                            }
                        }
                    }   
                }
                error!("Command {:?} sent but timed out before matching response received", command.to_bytes().as_slice());
                return Err(ImuError::ReadError("No response from IMU".to_string()));
            }
        }

        if let Ok(mut mode_guard) = self.mode.write() {
            *mode_guard = ImuMode::Read;
        }
        Ok(())
    }

    pub fn set_frequency(&self, frequency: ImuFrequency, timeout: Duration) -> Result<(), ImuError> {
        if let Ok(mut mode_guard) = self.mode.write() {
            *mode_guard = ImuMode::Write;
        }
        self.write_command(&UnlockCommand::new(), false, timeout)?;
        self.write_command(&SetFrequencyCommand::new(frequency), true, timeout)?;
        self.write_command(&SaveCommand::new(), false, timeout)?;
        Ok(())
    }

    pub fn set_baud_rate(&self, baud_rate: u32, timeout: Duration) -> Result<(), ImuError> {
        if let Ok(mut mode_guard) = self.mode.write() {
            *mode_guard = ImuMode::Write;
        }
        self.write_command(&UnlockCommand::new(), false, timeout)?;
        self.write_command(&SetBaudRateCommand::new(BaudRate::try_from(baud_rate).unwrap()), true, timeout)?;
        self.write_command(&SaveCommand::new(), false, timeout)?;
        Ok(())
    }
}

impl ImuReader for HiwonderReader {
    fn stop(&self) -> Result<(), ImuError> {
        self.reset()
    }

    fn get_data(&self) -> Result<ImuData, ImuError> {
        self.data.read()
            .map(|data| *data)
            .map_err(|e| ImuError::ReadError(format!("Data lock poisoned: {}", e)))
    }
}

impl Drop for HiwonderReader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
