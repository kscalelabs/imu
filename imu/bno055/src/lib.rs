mod registers;
use byteorder::{ByteOrder, LittleEndian};
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
use log::{error, warn};
pub use registers::OperationMode;
use registers::{
    AccelRegisters, ChipRegisters, Constants, EulerRegisters, GravityRegisters, GyroRegisters,
    LinearAccelRegisters, MagRegisters, QuaternionRegisters, RegisterPage, StatusRegisters,
};
use std::sync::mpsc;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;
use log::debug;

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
        debug!("BNO055::new - Opening I2C device on bus: {}", i2c_bus);
        let i2c = LinuxI2CDevice::new(i2c_bus, Constants::DefaultI2cAddr as u16)?;
        debug!("BNO055::new - I2C device opened");
        let mut bno = Bno055 { i2c };

        // Set page 0 before initialization
        debug!("BNO055::new - Setting page to Page0");
        bno.set_page(RegisterPage::Page0)?;

        // Verify we're talking to the right chip
        debug!("BNO055::new - Verifying chip ID");
        bno.verify_chip_id()?;

        // Reset the device
        debug!("BNO055::new - Resetting sensor");
        bno.reset()?;

        // Configure for NDOF mode (9-axis fusion)
        debug!("BNO055::new - Setting sensor mode to Ndof");
        bno.set_mode(OperationMode::Ndof)?;
        debug!("BNO055::new - Sensor initialization complete");

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

        // Synchronously initialize the BNO055 sensor.
        let imu = Bno055::new(i2c_bus)?;

        // Create a local "running" state for the thread.
        let running = Arc::new(RwLock::new(true));

        // Spawn a thread that continually reads sensor values using the already-initialized sensor.
        Self::start_reading_thread_with_imu(imu, Arc::clone(&data), Arc::clone(&running), command_rx);

        Ok(Bno055Reader {
            data,
            command_tx,
        })
    }

    // Remove the old asynchronous initialization method.
    // Instead, create a helper that takes an already-initialized sensor instance.
    fn start_reading_thread_with_imu(
        mut imu: Bno055,
        data: Arc<RwLock<BnoData>>,
        running: Arc<RwLock<bool>>,
        command_rx: mpsc::Receiver<ImuCommand>,
    ) {
        thread::spawn(move || {
            debug!("BNO055 reading thread started");
            while let Ok(guard) = running.read() {
                if !*guard {
                    break;
                }
                debug!("BNO055 reading thread iteration");

                // Process any pending commands
                if let Ok(command) = command_rx.try_recv() {
                    match command {
                        ImuCommand::SetMode(mode) => {
                            if let Err(e) = imu.set_mode(mode) {
                                error!("Failed to set mode: {}", e);
                            } else {
                                debug!("Set mode command processed");
                            }
                        }
                        ImuCommand::Reset => {
                            if let Err(e) = imu.reset() {
                                error!("Failed to reset: {}", e);
                            } else {
                                debug!("Reset command processed");
                            }
                        }
                        ImuCommand::Stop => {
                            if let Ok(mut guard) = running.write() {
                                *guard = false;
                            }
                            debug!("Stop command processed, exiting thread");
                            break;
                        }
                    }
                }

                let mut data_holder = BnoData::default();

                // Read sensor data with debug logging for each measurement.
                match imu.get_quaternion() {
                    Ok(quat) => {
                        debug!("Quaternion read successfully: {:?}", quat);
                        data_holder.quaternion = quat;
                    },
                    Err(e) => {
                        warn!("Failed to get quaternion: {}", e);
                    },
                }

                match imu.get_euler_angles() {
                    Ok(euler) => {
                        debug!("Euler angles read successfully: {:?}", euler);
                        data_holder.euler = euler;
                    },
                    Err(e) => {
                        warn!("Failed to get euler angles: {}", e);
                    },
                }

                match imu.get_accelerometer() {
                    Ok(accel) => {
                        debug!("Accelerometer read successfully: {:?}", accel);
                        data_holder.accelerometer = accel;
                    },
                    Err(e) => {
                        warn!("Failed to get accelerometer: {}", e);
                    },
                }

                match imu.get_gyroscope() {
                    Ok(gyro) => {
                        debug!("Gyroscope read successfully: {:?}", gyro);
                        data_holder.gyroscope = gyro;
                    },
                    Err(e) => {
                        warn!("Failed to get gyroscope: {}", e);
                    },
                }

                match imu.get_magnetometer() {
                    Ok(mag) => {
                        debug!("Magnetometer read successfully: {:?}", mag);
                        data_holder.magnetometer = mag;
                    },
                    Err(e) => {
                        warn!("Failed to get magnetometer: {}", e);
                    },
                }

                match imu.get_linear_acceleration() {
                    Ok(linear_accel) => {
                        debug!("Linear acceleration read successfully: {:?}", linear_accel);
                        data_holder.linear_acceleration = linear_accel;
                    },
                    Err(e) => {
                        warn!("Failed to get linear acceleration: {}", e);
                    },
                }

                match imu.get_gravity_vector() {
                    Ok(gravity) => {
                        debug!("Gravity vector read successfully: {:?}", gravity);
                        data_holder.gravity = gravity;
                    },
                    Err(e) => {
                        warn!("Failed to get gravity vector: {}", e);
                    },
                }

                match imu.get_temperature() {
                    Ok(temp) => {
                        debug!("Temperature read successfully: {:?}", temp);
                        data_holder.temperature = temp;
                    },
                    Err(e) => {
                        warn!("Failed to get temperature: {}", e);
                    },
                }

                match imu.get_calibration_status() {
                    Ok(status) => {
                        debug!("Calibration status read successfully: {:?}", status);
                        data_holder.calibration_status = status;
                    },
                    Err(e) => {
                        warn!("Failed to get calibration status: {}", e);
                    },
                }

                // Update shared data
                if let Ok(mut imu_data) = data.write() {
                    *imu_data = data_holder;
                } else {
                    warn!("Failed to write sensor data to shared state");
                }

                // Poll at roughly 100 Hz
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
