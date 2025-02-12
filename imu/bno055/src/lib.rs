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

#[derive(Debug)]
pub enum Error {
    I2c(i2cdev::linux::LinuxI2CError),
    InvalidChipId,
    CalibrationFailed,
    ReadError,
    WriteError,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::I2c(err) => write!(f, "I2C error: {}", err),
            Error::InvalidChipId => write!(f, "Invalid chip ID"),
            Error::CalibrationFailed => write!(f, "Calibration failed"),
            Error::ReadError => write!(f, "Read error"),
            Error::WriteError => write!(f, "Write error"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::I2c(err) => Some(err),
            _ => None,
        }
    }
}

impl From<i2cdev::linux::LinuxI2CError> for Error {
    fn from(err: i2cdev::linux::LinuxI2CError) -> Self {
        Error::I2c(err)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct EulerAngles {
    pub roll: f32,  // x-axis rotation
    pub pitch: f32, // y-axis rotation
    pub yaw: f32,   // z-axis rotation
}

#[derive(Debug, Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy)]
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

impl Default for BnoData {
    fn default() -> Self {
        BnoData {
            quaternion: Quaternion {
                w: 0.0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            euler: EulerAngles {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            },
            accelerometer: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            gyroscope: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            magnetometer: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            linear_acceleration: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            gravity: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            temperature: 0,
            calibration_status: 0,
        }
    }
}

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
    pub fn new(i2c_bus: &str) -> Result<Self, Error> {
        let data = Arc::new(RwLock::new(BnoData::default()));
        let (command_tx, command_rx) = mpsc::channel();

        // Synchronously initialize (calibrate) the IMU.
        // If this fails, the error is propagated immediately.
        let imu = Bno055::new(i2c_bus)?;

        // Spawn a thread that continuously reads sensor data using the initialized IMU.
        Self::start_reading_thread_with_imu(imu, Arc::clone(&data), command_rx);

        Ok(Bno055Reader { data, command_tx })
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

    pub fn set_mode(&self, mode: OperationMode) -> Result<(), Error> {
        self.command_tx
            .send(ImuCommand::SetMode(mode))
            .map_err(|_| Error::WriteError)
    }

    pub fn reset(&self) -> Result<(), Error> {
        self.command_tx
            .send(ImuCommand::Reset)
            .map_err(|_| Error::WriteError)
    }

    pub fn stop(&self) -> Result<(), Error> {
        self.command_tx
            .send(ImuCommand::Stop)
            .map_err(|_| Error::WriteError)
    }

    pub fn get_data(&self) -> Result<BnoData, Error> {
        self.data
            .read()
            .map(|data| *data)
            .map_err(|_| Error::ReadError)
    }
}

impl Drop for Bno055Reader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[derive(Debug)]
pub enum ImuCommand {
    SetMode(OperationMode),
    Reset,
    Stop,
}
