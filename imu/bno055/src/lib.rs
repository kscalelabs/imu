mod registers;
use byteorder::{ByteOrder, LittleEndian};
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
use log::error;
pub use registers::OperationMode;
use registers::{
    AccelRegisters, CalibrationRegisters, ChipRegisters, ConfigRegisters, Constants,
    EulerRegisters, GravityRegisters, GyroRegisters, LinearAccelRegisters, MagRegisters,
    QuaternionRegisters, RegisterPage, StatusRegisters,
};
use std::thread;
use std::time::Duration;

#[derive(Debug)]
pub enum Error {
    I2c(i2cdev::linux::LinuxI2CError),
    InvalidChipId,
    CalibrationFailed,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::I2c(err) => write!(f, "I2C error: {}", err),
            Error::InvalidChipId => write!(f, "Invalid chip ID"),
            Error::CalibrationFailed => write!(f, "Calibration failed"),
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
        for i in 0..8 {
            buf[i] = self
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
        for i in 0..6 {
            buf[i] = self
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
        for i in 0..6 {
            buf[i] = self
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
        for i in 0..6 {
            buf[i] = self
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
        for i in 0..6 {
            buf[i] = self
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
        for i in 0..6 {
            buf[i] = self
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
        for i in 0..6 {
            buf[i] = self
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
