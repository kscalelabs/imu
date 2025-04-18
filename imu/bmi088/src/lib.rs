use byteorder::{ByteOrder, LittleEndian};
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
use log::{debug, error, warn};
use std::sync::{mpsc, Arc, RwLock};
use std::thread;
use std::time::Duration;
// Import types from parent imu crate
use imu::{ImuData as BaseImuData, ImuError as BaseImuError, ImuReader as BaseImuReader, Quaternion as BaseQuaternion, Vector3 as BaseVector3};

mod registers;
use registers::{AccelRange, AccelRegisters, Constants, GyroRange, GyroRegisters};

// Add these constants at the top of the file
pub const ACCEL_ADDR: u8 = 0x18; // Default BMI088 accelerometer address
pub const GYRO_ADDR: u8 = 0x68; // Default BMI088 gyroscope address

/// 3D vector type. (Local)
#[derive(Debug, Clone, Copy, Default)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Quaternion type. (Local - Unused by BMI088 data but keep for consistency?)
#[derive(Debug, Clone, Copy, Default)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Euler angles (in degrees). (Local - Unused by BMI088 data)
#[derive(Debug, Clone, Copy, Default)]
pub struct EulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// Sensor data struct. (Local)
#[derive(Debug, Clone, Copy, Default)] // Added Default
pub struct Bmi088Data {
    pub accelerometer: Vector3,
    pub gyroscope: Vector3,
    pub temperature: i8,
}

/// Errors for BMI088 operations. (Local)
#[derive(Debug)]
pub enum Error {
    I2c(i2cdev::linux::LinuxI2CError),
    InvalidChipId,
    ReadError,
    WriteError,
}

// Map local Error to BaseImuError
impl From<Error> for BaseImuError {
    fn from(err: Error) -> Self {
        match err {
            Error::I2c(e) => BaseImuError::DeviceError(format!("I2C error: {}", e)),
            Error::InvalidChipId => BaseImuError::ConfigurationError("Invalid chip ID".to_string()),
            Error::ReadError => BaseImuError::ReadError("BMI088 read error".to_string()),
            Error::WriteError => BaseImuError::WriteError("BMI088 write error".to_string()),
        }
    }
}

// Map PoisonError to BaseImuError
impl<T> From<std::sync::PoisonError<T>> for BaseImuError {
    fn from(err: std::sync::PoisonError<T>) -> Self {
        BaseImuError::LockError(format!("Mutex poisoned: {}", err))
    }
}

// Map SendError to BaseImuError
impl From<mpsc::SendError<ImuCommand>> for BaseImuError {
    fn from(err: mpsc::SendError<ImuCommand>) -> Self {
        BaseImuError::CommandSendError(format!("Failed to send command: {}", err))
    }
}

/// Low-level BMI088 driver.
pub struct Bmi088 {
    accel_i2c: LinuxI2CDevice,
    gyro_i2c: LinuxI2CDevice,
    accel_range: AccelRange,
    gyro_range: GyroRange,
}

impl Bmi088 {
    /// Initializes the BMI088 sensor on the given I2C bus.
    pub fn new(i2c_path: &str) -> Result<Self, Error> {
        debug!("Initializing Bmi088...");

        let mut accel_i2c = LinuxI2CDevice::new(i2c_path, Constants::AccelI2cAddr as u16)?;
        let gyro_i2c = LinuxI2CDevice::new(i2c_path, Constants::GyroI2cAddr as u16)?;

        // Verify accelerometer chip ID.
        let chip_id = accel_i2c.smbus_read_byte_data(AccelRegisters::ChipId as u8)?;
        if chip_id != Constants::AccelChipIdValue as u8 {
            return Err(Error::InvalidChipId);
        }
        debug!("Bmi088 Accel chip ID verified: 0x{:02X}", chip_id);

        // Soft reset, power-up, and configure accelerometer.
        accel_i2c.smbus_write_byte_data(
            AccelRegisters::SoftReset as u8,
            Constants::SoftResetCmd as u8,
        )?;
        thread::sleep(Duration::from_millis(50));
        accel_i2c.smbus_write_byte_data(AccelRegisters::PowerCtrl as u8, 0x04)?;
        accel_i2c.smbus_write_byte_data(AccelRegisters::AccConf as u8, 0x80)?;
        accel_i2c.smbus_write_byte_data(AccelRegisters::AccRange as u8, AccelRange::G3 as u8)?;

        // Configure gyroscope.
        let mut gyro_i2c = gyro_i2c; // no mutable needed outside this scope
        gyro_i2c.smbus_write_byte_data(GyroRegisters::PowerMode as u8, 0x00)?;
        gyro_i2c.smbus_write_byte_data(GyroRegisters::Range as u8, GyroRange::Dps2000 as u8)?;
        gyro_i2c.smbus_write_byte_data(GyroRegisters::Bandwidth as u8, 0x07)?;

        Ok(Bmi088 {
            accel_i2c,
            gyro_i2c,
            accel_range: AccelRange::G3,
            gyro_range: GyroRange::Dps2000,
        })
    }

    /// Reads raw accelerometer data without remapping.
    pub fn read_raw_accelerometer(&mut self) -> Result<Vector3, Error> {
        let mut buf = [0u8; 6];
        for i in 0..6 {
            buf[i] = self
                .accel_i2c
                .smbus_read_byte_data(AccelRegisters::AccelXLsb as u8 + i as u8)?;
        }
        let scale = match self.accel_range {
            AccelRange::G3 => 3.0 / 32768.0,
            AccelRange::G6 => 6.0 / 32768.0,
            AccelRange::G12 => 12.0 / 32768.0,
            AccelRange::G24 => 24.0 / 32768.0,
        };
        let raw_x = (LittleEndian::read_i16(&buf[0..2]) as f32) * scale;
        let raw_y = (LittleEndian::read_i16(&buf[2..4]) as f32) * scale;
        let raw_z = (LittleEndian::read_i16(&buf[4..6]) as f32) * scale;
        Ok(Vector3 {
            x: raw_x,
            y: raw_y,
            z: raw_z,
        })
    }

    /// Reads raw gyroscope data without remapping.
    pub fn read_raw_gyroscope(&mut self) -> Result<Vector3, Error> {
        let mut buf = [0u8; 6];
        for i in 0..6 {
            buf[i] = self
                .gyro_i2c
                .smbus_read_byte_data(GyroRegisters::XLsb as u8 + i as u8)?;
        }
        let scale = match self.gyro_range {
            GyroRange::Dps2000 => 2000.0 / 32768.0,
            GyroRange::Dps1000 => 1000.0 / 32768.0,
            GyroRange::Dps500 => 500.0 / 32768.0,
            GyroRange::Dps250 => 250.0 / 32768.0,
            GyroRange::Dps125 => 125.0 / 32768.0,
        };
        let raw_x = (LittleEndian::read_i16(&buf[0..2]) as f32) * scale;
        let raw_y = (LittleEndian::read_i16(&buf[2..4]) as f32) * scale;
        let raw_z = (LittleEndian::read_i16(&buf[4..6]) as f32) * scale;
        Ok(Vector3 {
            x: raw_x,
            y: raw_y,
            z: raw_z,
        })
    }

    /// Reads temperature (in °C) from the accelerometer.
    pub fn read_temperature(&mut self) -> Result<i8, Error> {
        let msb = self
            .accel_i2c
            .smbus_read_byte_data(AccelRegisters::TempMsb as u8)? as i16;
        let lsb = self
            .accel_i2c
            .smbus_read_byte_data(AccelRegisters::TempLsb as u8)? as i16;
        let temp_raw = (msb * 8) + (lsb / 32);
        Ok(((temp_raw as f32) * 0.125 + 23.0) as i8)
    }
}

/// BMI088Reader runs a background thread to update sensor data continuously.
pub struct Bmi088Reader {
    data: Arc<RwLock<Bmi088Data>>,
    command_tx: mpsc::Sender<ImuCommand>,
    running: Arc<RwLock<bool>>,
}

/// Commands sent to the reading thread.
#[derive(Debug)]
pub enum ImuCommand {
    SetAccelRange(AccelRange),
    SetGyroRange(GyroRange),
    Reset,
    Stop,
}

impl Bmi088Reader {
    /// Creates a new BMI088Reader.
    pub fn new(i2c_bus: &str) -> Result<Self, Error> {
        debug!("Initializing Bmi088 Reader...");
        let data = Arc::new(RwLock::new(Bmi088Data::default()));
        let running = Arc::new(RwLock::new(true));
        let (command_tx, command_rx) = mpsc::channel();
        let reader = Bmi088Reader {
            data: Arc::clone(&data),
            command_tx,
            running: Arc::clone(&running),
        };
        reader.start_reading_thread(i2c_bus.to_string(), command_rx, Arc::clone(&running))?;
        Ok(reader)
    }

    fn start_reading_thread(
        &self,
        i2c_bus: String,
        command_rx: mpsc::Receiver<ImuCommand>,
        running: Arc<RwLock<bool>>,
    ) -> Result<(), Error> {
        thread::spawn(move || {
            debug!("BMI088 reading thread started");
            match Bmi088::new(&i2c_bus) {
                Ok(mut imu) => {
                    loop {
                        if let Ok(guard) = running.read() {
                            if !*guard { break; }
                        } else {
                            error!("BMI088 reader: Failed to read running flag");
                            break;
                        }

                        if let Ok(command) = command_rx.try_recv() {
                           match command {
                                ImuCommand::SetAccelRange(range) => {
                                    warn!("SetAccelRange command not fully implemented");
                                }
                                ImuCommand::SetGyroRange(range) => {
                                    warn!("SetGyroRange command not fully implemented");
                                }
                                ImuCommand::Reset => {
                                    warn!("Reset command not fully implemented");
                                }
                                ImuCommand::Stop => break,
                            }
                        }

                        let mut data_holder = Bmi088Data::default();
                        match imu.read_raw_accelerometer() {
                            Ok(accel) => data_holder.accelerometer = accel,
                            Err(e) => warn!("Failed to read accelerometer: {}", e),
                        }
                        match imu.read_raw_gyroscope() {
                            Ok(gyro) => data_holder.gyroscope = gyro,
                            Err(e) => warn!("Failed to read gyroscope: {}", e),
                        }
                         match imu.read_temperature() {
                            Ok(temp) => data_holder.temperature = temp,
                            Err(e) => warn!("Failed to read temperature: {}", e),
                        }

                        if let Ok(mut shared_data) = self.data.write() {
                            *shared_data = data_holder;
                        }

                        thread::sleep(Duration::from_millis(10));
                    }
                }
                Err(e) => {
                    error!("Failed to initialize BMI088 in reading thread: {}", e);
                    if let Ok(mut guard) = running.write() {
                         *guard = false;
                    }
                }
            }
            debug!("BMI088 reading thread exiting");
        });
        Ok(())
    }

    pub fn get_data_local(&self) -> Result<Bmi088Data, BaseImuError> {
        Ok(self.data.read()?.clone())
    }

    pub fn stop_local(&self) -> Result<(), BaseImuError> {
        if let Ok(mut running) = self.running.write() {
             *running = false;
        }
        let _ = self.command_tx.send(ImuCommand::Stop);
        Ok(())
    }

    pub fn reset(&self) -> Result<(), BaseImuError> {
        self.command_tx.send(ImuCommand::Reset)?;
        Ok(())
    }
}

impl Drop for Bmi088Reader {
    fn drop(&mut self) {
        let _ = self.stop_local();
    }
}

impl BaseImuReader for Bmi088Reader {
    fn get_data(&self) -> Result<BaseImuData, BaseImuError> {
        let local_data = self.data.read()?;

        let base_data = BaseImuData {
            accelerometer: Some(BaseVector3 { x: local_data.accelerometer.x, y: local_data.accelerometer.y, z: local_data.accelerometer.z }),
            gyroscope: Some(BaseVector3 { x: local_data.gyroscope.x, y: local_data.gyroscope.y, z: local_data.gyroscope.z }),
            magnetometer: None,
            quaternion: None,
            euler: None,
            linear_acceleration: None,
            gravity: None,
            temperature: Some(local_data.temperature as f32),
        };
        Ok(base_data)
    }

    fn stop(&self) -> Result<(), BaseImuError> {
        self.stop_local()
    }
}
