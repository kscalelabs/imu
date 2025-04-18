mod registers;
use byteorder::{ByteOrder, LittleEndian};
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
use log::{debug, error, warn};
pub use registers::OperationMode;
use registers::{
    AccelRegisters, ChipRegisters, Constants, EulerRegisters, GravityRegisters, GyroRegisters,
    LinearAccelRegisters, MagRegisters, QuaternionRegisters, RegisterPage, StatusRegisters,
};
use std::sync::mpsc;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;
// Import types from parent imu crate
use imu::{ImuData as BaseImuData, ImuError as BaseImuError, ImuReader as BaseImuReader, Quaternion as BaseQuaternion, Vector3 as BaseVector3};

// Local Error type
#[derive(Debug)]
pub enum Error {
    I2c(i2cdev::linux::LinuxI2CError),
    InvalidChipId,
    CalibrationFailed,
    ReadError,
    WriteError,
}

// Map local Error to BaseImuError
impl From<Error> for BaseImuError {
    fn from(err: Error) -> Self {
        match err {
            Error::I2c(e) => BaseImuError::DeviceError(format!("I2C error: {}", e)),
            Error::InvalidChipId => BaseImuError::ConfigurationError("Invalid chip ID".to_string()),
            Error::CalibrationFailed => BaseImuError::ConfigurationError("Calibration failed".to_string()),
            Error::ReadError => BaseImuError::ReadError("BNO055 read error".to_string()),
            Error::WriteError => BaseImuError::WriteError("BNO055 write error".to_string()),
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

// Local data structures (keep for internal use)
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct EulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy, Default)] // Added Default
pub struct BnoData {
    pub quaternion: Quaternion,
    pub euler: EulerAngles,
    pub accelerometer: Vector3,
    pub gyroscope: Vector3,
    pub magnetometer: Vector3,
    pub linear_acceleration: Vector3,
    pub gravity: Vector3,
    pub temperature: i8,
    pub calibration_status: u8,
}

// Remove explicit Default impl as derive is added
// impl Default for BnoData { ... }

// Bno055 struct (low-level driver) remains the same
pub struct Bno055 {
    i2c: LinuxI2CDevice,
}

impl Bno055 {
    /// Creates a new BNO055 device instance using the specified I2C bus.
    ///
    /// # Arguments
    /// * `i2c_bus` - The I2C bus path (e.g., "/dev/i2c-1")
    pub fn new(i2c_bus: &str) -> Result<Self, Error> {
        let i2c = LinuxI2CDevice::new(i2c_bus, Constants::DefaultI2cAddr as u16)?;
        let mut bno = Bno055 { i2c };

        // Set page 0 before initialization
        bno.set_page(RegisterPage::Page0)?;

        // Verify we're talking to the right chip
        bno.verify_chip_id()?;

        // Reset the device
        bno.reset()?;

        // Configure for NDOF mode (9-axis fusion)
        bno.set_mode(OperationMode::Ndof)?;

        Ok(bno)
    }

    fn set_page(&mut self, page: RegisterPage) -> Result<(), Error> {
        self.i2c
            .smbus_write_byte_data(ChipRegisters::PageId as u8, page as u8)
            .map_err(Error::I2c)
    }

    fn verify_chip_id(&mut self) -> Result<(), Error> {
        self.set_page(RegisterPage::Page0)?;
        let chip_id = self.i2c.smbus_read_byte_data(ChipRegisters::ChipId as u8)?;
        if Constants::ChipId as u8 != chip_id {
            error!("Invalid chip ID. Expected 0xA0, got {:#x}", chip_id);
            return Err(Error::InvalidChipId);
        }
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        self.set_page(RegisterPage::Page0)?;
        self.i2c
            .smbus_write_byte_data(StatusRegisters::SysTrigger as u8, 0x20)?;
        thread::sleep(Duration::from_millis(650));
        Ok(())
    }

    /// Returns the current orientation as a quaternion.
    /// The quaternion values are normalized and unitless.
    pub fn get_quaternion(&mut self) -> Result<Quaternion, Error> {
        self.set_page(RegisterPage::Page0)?;
        let mut buf = [0u8; 8];

        // Read all quaternion data at once
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = self
                .i2c
                .smbus_read_byte_data((QuaternionRegisters::WLsb as u8) + i as u8)?;
        }

        let scale = 1.0 / ((1 << 14) as f32);
        Ok(Quaternion {
            w: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            x: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            y: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
            z: (LittleEndian::read_i16(&buf[6..8]) as f32) * scale,
        })
    }

    /// Returns the current orientation in Euler angles.
    /// All angles (roll, pitch, yaw) are in degrees.
    pub fn get_euler_angles(&mut self) -> Result<EulerAngles, Error> {
        self.set_page(RegisterPage::Page0)?;
        let mut buf = [0u8; 6];

        // Read all euler angle data at once
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = self
                .i2c
                .smbus_read_byte_data((EulerRegisters::HLsb as u8) + i as u8)?;
        }

        // Convert to degrees (scale factor is 16)
        let scale = 1.0 / 16.0;
        Ok(EulerAngles {
            yaw: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            roll: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            pitch: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
        })
    }

    /// Returns the linear acceleration vector with gravity compensation.
    /// All components (x, y, z) are in meters per second squared (m/s²).
    pub fn get_linear_acceleration(&mut self) -> Result<Vector3, Error> {
        self.set_page(RegisterPage::Page0)?;
        let mut buf = [0u8; 6];

        // Read all linear acceleration data at once
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = self
                .i2c
                .smbus_read_byte_data((LinearAccelRegisters::XLsb as u8) + i as u8)?;
        }

        // Convert to m/s² (scale factor is 100)
        let scale = 1.0 / 100.0;
        Ok(Vector3 {
            x: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            y: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            z: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
        })
    }

    /// Returns the gravity vector with linear acceleration compensation.
    /// All components (x, y, z) are in meters per second squared (m/s²).
    pub fn get_gravity_vector(&mut self) -> Result<Vector3, Error> {
        self.set_page(RegisterPage::Page0)?;
        let mut buf = [0u8; 6];

        // Read all gravity vector data at once
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = self
                .i2c
                .smbus_read_byte_data((GravityRegisters::XLsb as u8) + i as u8)?;
        }

        let scale = 1.0 / 100.0;
        Ok(Vector3 {
            x: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            y: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            z: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
        })
    }

    /// Sets the operation mode of the BNO055.
    ///
    /// # Arguments
    /// * `mode` - The operation mode to set.
    pub fn set_mode(&mut self, mode: OperationMode) -> Result<(), Error> {
        self.set_page(RegisterPage::Page0)?;
        self.i2c
            .smbus_write_byte_data(StatusRegisters::OprMode as u8, mode as u8)?;
        // Wait for mode switch to complete
        thread::sleep(Duration::from_millis(20));
        Ok(())
    }

    /// Returns the raw accelerometer readings including gravity.
    /// All components (x, y, z) are in meters per second squared (m/s²).
    pub fn get_accelerometer(&mut self) -> Result<Vector3, Error> {
        self.set_page(RegisterPage::Page0)?;
        let mut buf = [0u8; 6];

        // Read all accelerometer data at once
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = self
                .i2c
                .smbus_read_byte_data((AccelRegisters::XLsb as u8) + i as u8)?;
        }

        // Convert to m/s² (scale factor is 100)
        let scale = 1.0 / 100.0;
        Ok(Vector3 {
            x: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            y: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            z: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
        })
    }

    /// Returns the magnetometer readings.
    /// All components (x, y, z) are in microTesla (µT).
    pub fn get_magnetometer(&mut self) -> Result<Vector3, Error> {
        self.set_page(RegisterPage::Page0)?;
        let mut buf = [0u8; 6];

        // Read all magnetometer data at once
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = self
                .i2c
                .smbus_read_byte_data((MagRegisters::XLsb as u8) + i as u8)?;
        }

        // Convert to microTesla
        let scale = 1.0 / 16.0;
        Ok(Vector3 {
            x: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            y: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            z: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
        })
    }

    /// Returns the gyroscope readings.
    /// All components (x, y, z) are in degrees per second (°/s).
    pub fn get_gyroscope(&mut self) -> Result<Vector3, Error> {
        self.set_page(RegisterPage::Page0)?;
        let mut buf = [0u8; 6];

        // Read all gyroscope data at once
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = self
                .i2c
                .smbus_read_byte_data((GyroRegisters::XLsb as u8) + i as u8)?;
        }

        // Convert to degrees per second
        let scale = 1.0 / 16.0;
        Ok(Vector3 {
            x: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            y: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            z: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
        })
    }

    /// Returns the chip temperature.
    /// Temperature is in degrees Celsius (°C).
    pub fn get_temperature(&mut self) -> Result<i8, Error> {
        self.set_page(RegisterPage::Page0)?;
        let temp = self
            .i2c
            .smbus_read_byte_data(StatusRegisters::Temperature as u8)? as i8;
        Ok(temp)
    }

    /// Returns the calibration status byte.
    /// Bits 5-4: gyroscope (0-3)
    /// Bits 3-2: accelerometer (0-3)
    /// Bits 1-0: magnetometer (0-3)
    /// For each sensor, 0 = uncalibrated, 3 = fully calibrated
    pub fn get_calibration_status(&mut self) -> Result<u8, Error> {
        self.set_page(RegisterPage::Page0)?;
        let status = self
            .i2c
            .smbus_read_byte_data(StatusRegisters::CalibStat as u8)?;
        Ok(status)
    }
}

pub struct Bno055Reader {
    data: Arc<RwLock<BnoData>>,
    command_tx: mpsc::Sender<ImuCommand>,
}

impl Bno055Reader {
    /// Constructor returns local Error type for now.
    pub fn new(i2c_bus: &str) -> Result<Self, Error> {
        let imu = Bno055::new(i2c_bus)?;
        let data = Arc::new(RwLock::new(BnoData::default()));
        let (command_tx, command_rx) = mpsc::channel();

        let reader = Bno055Reader {
            data: Arc::clone(&data),
            command_tx,
        };

        // Start the background thread
        Self::start_reading_thread_with_imu(imu, data, command_rx);

        Ok(reader)
    }

    fn start_reading_thread_with_imu(
        mut imu: Bno055,
        data: Arc<RwLock<BnoData>>,
        command_rx: mpsc::Receiver<ImuCommand>,
    ) {
        thread::spawn(move || {
            debug!("BNO055 reading thread started");
            loop {
                // Process any pending commands
                if let Ok(command) = command_rx.try_recv() {
                    match command {
                        ImuCommand::SetMode(mode) => {
                            if let Err(e) = imu.set_mode(mode) {
                                error!("Failed to set mode: {}", e);
                            }
                        }
                        ImuCommand::Reset => {
                            if let Err(e) = imu.reset() {
                                error!("Failed to reset: {}", e);
                            }
                        }
                        ImuCommand::Stop => break,
                    }
                }

                // Read sensor data and update shared data
                let mut data_holder = BnoData::default();

                if let Ok(quat) = imu.get_quaternion() {
                    data_holder.quaternion = quat;
                } else {
                    warn!("Failed to get quaternion");
                }

                if let Ok(euler) = imu.get_euler_angles() {
                    data_holder.euler = euler;
                } else {
                    warn!("Failed to get euler angles");
                }

                if let Ok(accel) = imu.get_accelerometer() {
                    data_holder.accelerometer = accel;
                } else {
                    warn!("Failed to get accelerometer");
                }

                if let Ok(gyro) = imu.get_gyroscope() {
                    data_holder.gyroscope = gyro;
                } else {
                    warn!("Failed to get gyroscope");
                }

                if let Ok(mag) = imu.get_magnetometer() {
                    data_holder.magnetometer = mag;
                } else {
                    warn!("Failed to get magnetometer");
                }

                if let Ok(linear_accel) = imu.get_linear_acceleration() {
                    data_holder.linear_acceleration = linear_accel;
                } else {
                    warn!("Failed to get linear acceleration");
                }

                if let Ok(gravity) = imu.get_gravity_vector() {
                    data_holder.gravity = gravity;
                } else {
                    warn!("Failed to get gravity vector");
                }

                if let Ok(temp) = imu.get_temperature() {
                    data_holder.temperature = temp;
                } else {
                    warn!("Failed to get temperature");
                }

                if let Ok(status) = imu.get_calibration_status() {
                    data_holder.calibration_status = status;
                } else {
                    warn!("Failed to get calibration status");
                }

                // Update shared data
                if let Ok(mut imu_data) = data.write() {
                    *imu_data = data_holder;
                }

                // IMU sends data at 100 Hz
                thread::sleep(Duration::from_millis(10));
            }
            debug!("BNO055 reading thread exiting");
        });
    }

    pub fn set_mode(&self, mode: OperationMode) -> Result<(), BaseImuError> {
        self.command_tx.send(ImuCommand::SetMode(mode))?;
        Ok(())
    }

    pub fn reset(&self) -> Result<(), BaseImuError> {
        self.command_tx.send(ImuCommand::Reset)?;
        Ok(())
    }

    // Rename inherent stop to avoid conflict
    pub fn stop_local(&self) -> Result<(), BaseImuError> {
        self.command_tx.send(ImuCommand::Stop)?;
        Ok(())
    }

    // Rename inherent get_data to avoid conflict
    pub fn get_data_local(&self) -> Result<BnoData, BaseImuError> {
        Ok(self.data.read()?.clone()) // Propagate lock errors
    }
}

// Implement Drop using stop_local
impl Drop for Bno055Reader {
    fn drop(&mut self) {
        let _ = self.stop_local(); // Ignore error on drop
    }
}

// Implement the BaseImuReader trait
impl BaseImuReader for Bno055Reader {
    /// Retrieves the latest available IMU data, converted to the standard format.
    fn get_data(&self) -> Result<BaseImuData, BaseImuError> {
        let local_data = self.data.read()?;

        // Convert local BnoData to BaseImuData
        let base_data = BaseImuData {
            accelerometer: Some(BaseVector3 { x: local_data.accelerometer.x, y: local_data.accelerometer.y, z: local_data.accelerometer.z }),
            gyroscope: Some(BaseVector3 { x: local_data.gyroscope.x, y: local_data.gyroscope.y, z: local_data.gyroscope.z }),
            magnetometer: Some(BaseVector3 { x: local_data.magnetometer.x, y: local_data.magnetometer.y, z: local_data.magnetometer.z }),
            quaternion: Some(BaseQuaternion { w: local_data.quaternion.w, x: local_data.quaternion.x, y: local_data.quaternion.y, z: local_data.quaternion.z }),
            // Use local euler data (roll, pitch, yaw) for BaseVector3 (x, y, z)
            euler: Some(BaseVector3 { x: local_data.euler.roll, y: local_data.euler.pitch, z: local_data.euler.yaw }),
            linear_acceleration: Some(BaseVector3 { x: local_data.linear_acceleration.x, y: local_data.linear_acceleration.y, z: local_data.linear_acceleration.z }),
            gravity: Some(BaseVector3 { x: local_data.gravity.x, y: local_data.gravity.y, z: local_data.gravity.z }),
            temperature: Some(local_data.temperature as f32),
        };
        Ok(base_data)
    }

    /// Stops the background reading thread.
    fn stop(&self) -> Result<(), BaseImuError> {
        self.stop_local()
    }

    // Note: If the BaseImuReader trait defines other methods like `reset` or `set_mode`,
    // they would need to be implemented here, likely by calling the inherent methods.
    // Example:
    // fn reset(&self) -> Result<(), BaseImuError> { self.reset() }
}

#[derive(Debug)]
pub enum ImuCommand {
    SetMode(OperationMode),
    Reset,
    Stop,
}
