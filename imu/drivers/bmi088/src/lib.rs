use byteorder::{ByteOrder, LittleEndian};
use i2cdev::core::I2CDevice; // Add this import
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
pub use imu_traits::{ImuData, ImuError, ImuReader, Vector3};
use log::{debug, error, warn};
use std::sync::{mpsc, Arc, RwLock};
use std::thread;
use std::time::Duration;

mod registers;
use registers::{AccelRange, AccelRegisters, Constants, GyroRange, GyroRegisters};

// Add these constants at the top of the file
pub const ACCEL_ADDR: u8 = 0x18; // Default BMI088 accelerometer address
pub const GYRO_ADDR: u8 = 0x68; // Default BMI088 gyroscope address

pub struct BmiI2CError(LinuxI2CError);

impl From<LinuxI2CError> for BmiI2CError {
    fn from(err: LinuxI2CError) -> Self {
        BmiI2CError(err)
    }
}

impl From<BmiI2CError> for ImuError {
    fn from(err: BmiI2CError) -> Self {
        ImuError::DeviceError(err.0.to_string())
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
    pub fn new(i2c_path: &str) -> Result<Self, ImuError> {
        debug!("Initializing Bmi088...");

        let mut accel_i2c =
            LinuxI2CDevice::new(i2c_path, Constants::AccelI2cAddr as u16).map_err(BmiI2CError)?;
        let gyro_i2c =
            LinuxI2CDevice::new(i2c_path, Constants::GyroI2cAddr as u16).map_err(BmiI2CError)?;

        // Verify accelerometer chip ID.
        let chip_id = accel_i2c
            .smbus_read_byte_data(AccelRegisters::ChipId as u8)
            .map_err(BmiI2CError)?;
        if chip_id != Constants::AccelChipIdValue as u8 {
            return Err(ImuError::DeviceError("Invalid chip ID".to_string()));
        }
        debug!("Bmi088 Accel chip ID verified: 0x{:02X}", chip_id);

        // Soft reset, power-up, and configure accelerometer.
        accel_i2c
            .smbus_write_byte_data(
                AccelRegisters::SoftReset as u8,
                Constants::SoftResetCmd as u8,
            )
            .map_err(BmiI2CError)?;
        thread::sleep(Duration::from_millis(50));
        accel_i2c
            .smbus_write_byte_data(AccelRegisters::PowerCtrl as u8, 0x04)
            .map_err(BmiI2CError)?;
        accel_i2c
            .smbus_write_byte_data(AccelRegisters::AccConf as u8, 0x80)
            .map_err(BmiI2CError)?;
        accel_i2c
            .smbus_write_byte_data(AccelRegisters::AccRange as u8, AccelRange::G3 as u8)
            .map_err(BmiI2CError)?;

        // Configure gyroscope.
        let mut gyro_i2c = gyro_i2c; // no mutable needed outside this scope
        gyro_i2c
            .smbus_write_byte_data(GyroRegisters::PowerMode as u8, 0x00)
            .map_err(BmiI2CError)?;
        gyro_i2c
            .smbus_write_byte_data(GyroRegisters::Range as u8, GyroRange::Dps2000 as u8)
            .map_err(BmiI2CError)?;
        gyro_i2c
            .smbus_write_byte_data(GyroRegisters::Bandwidth as u8, 0x07)
            .map_err(BmiI2CError)?;

        Ok(Bmi088 {
            accel_i2c,
            gyro_i2c,
            accel_range: AccelRange::G3,
            gyro_range: GyroRange::Dps2000,
        })
    }

    /// Reads raw accelerometer data without remapping.
    pub fn read_raw_accelerometer(&mut self) -> Result<Vector3, ImuError> {
        let mut buf = [0u8; 6];
        for i in 0..6 {
            buf[i] = self
                .accel_i2c
                .smbus_read_byte_data(AccelRegisters::AccelXLsb as u8 + i as u8)
                .map_err(BmiI2CError)?;
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
    pub fn read_raw_gyroscope(&mut self) -> Result<Vector3, ImuError> {
        let mut buf = [0u8; 6];
        for i in 0..6 {
            buf[i] = self
                .gyro_i2c
                .smbus_read_byte_data(GyroRegisters::XLsb as u8 + i as u8)
                .map_err(BmiI2CError)?;
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
    pub fn read_temperature(&mut self) -> Result<i8, ImuError> {
        let msb = self
            .accel_i2c
            .smbus_read_byte_data(AccelRegisters::TempMsb as u8)
            .map_err(BmiI2CError)? as i16;
        let lsb = self
            .accel_i2c
            .smbus_read_byte_data(AccelRegisters::TempLsb as u8)
            .map_err(BmiI2CError)? as i16;
        let temp_raw = (msb * 8) + (lsb / 32);
        Ok(((temp_raw as f32) * 0.125 + 23.0) as i8)
    }
}

/// BMI088Reader runs a background thread to update sensor data continuously.
pub struct Bmi088Reader {
    data: Arc<RwLock<ImuData>>,
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
    pub fn new(i2c_bus: &str) -> Result<Self, ImuError> {
        debug!("Initializing Bmi088 Reader...");
        let data = Arc::new(RwLock::new(ImuData::default()));
        let running = Arc::new(RwLock::new(true));
        let (command_tx, command_rx) = mpsc::channel();
        let reader = Bmi088Reader {
            data: Arc::clone(&data),
            command_tx,
            running: Arc::clone(&running),
        };
        reader.start_reading_thread(i2c_bus, command_rx)?;
        Ok(reader)
    }

    fn start_reading_thread(
        &self,
        i2c_bus: &str,
        command_rx: mpsc::Receiver<ImuCommand>,
    ) -> Result<(), ImuError> {
        let data = Arc::clone(&self.data);
        let running = Arc::clone(&self.running);
        let i2c_bus = i2c_bus.to_string();
        let (tx, rx) = mpsc::channel();

        thread::spawn(move || {
            let init_result = Bmi088::new(&i2c_bus);
            if let Err(e) = init_result {
                error!("Failed to initialize BMI088: {}", e);
                let _ = tx.send(Err(e));
                return;
            }
            let mut imu = init_result.unwrap();
            let _ = tx.send(Ok(()));

            while let Ok(guard) = running.read() {
                if !*guard {
                    break;
                }

                // Handle incoming commands.
                if let Ok(cmd) = command_rx.try_recv() {
                    match cmd {
                        ImuCommand::SetAccelRange(range) => {
                            imu.accel_range = range;
                            let _ = imu
                                .accel_i2c
                                .smbus_write_byte_data(AccelRegisters::AccRange as u8, range as u8);
                        }
                        ImuCommand::SetGyroRange(range) => {
                            imu.gyro_range = range;
                            let _ = imu
                                .gyro_i2c
                                .smbus_write_byte_data(GyroRegisters::Range as u8, range as u8);
                        }
                        ImuCommand::Reset => {
                            // Reinitialize if needed.
                            let _ = imu = Bmi088::new(&i2c_bus).unwrap();
                        }
                        ImuCommand::Stop => {
                            if let Ok(mut w) = running.write() {
                                *w = false;
                            }
                            break;
                        }
                    }
                }

                let mut sensor_data = ImuData::default();
                if let Ok(accel) = imu.read_raw_accelerometer() {
                    sensor_data.accelerometer = Some(accel);
                } else {
                    warn!("Failed to read accelerometer");
                }
                if let Ok(gyro) = imu.read_raw_gyroscope() {
                    sensor_data.gyroscope = Some(gyro);
                } else {
                    warn!("Failed to read gyroscope");
                }
                if let Ok(temp) = imu.read_temperature() {
                    sensor_data.temperature = Some(temp as f32);
                }

                if let Ok(mut shared) = data.write() {
                    *shared = sensor_data;
                }
                thread::sleep(Duration::from_millis(10));
            }
        });

        rx.recv()?
    }

    /// Resets the BMI088 sensor.
    pub fn reset(&self) -> Result<(), ImuError> {
        self.command_tx.send(ImuCommand::Reset)?;
        Ok(())
    }

    /// Sets the mode for BMI088.
    /// BMI088 does not support the mode configuration like BNO055, so this is a no-op.
    pub fn set_mode(&self, _mode: u8) -> Result<(), ImuError> {
        Ok(())
    }
}

impl ImuReader for Bmi088Reader {
    /// Returns the most recent sensor data.
    fn get_data(&self) -> Result<ImuData, ImuError> {
        Ok(self.data.read().map(|data| *data)?)
    }

    /// Stops the reading thread.
    fn stop(&self) -> Result<(), ImuError> {
        self.command_tx.send(ImuCommand::Stop)?;
        Ok(())
    }
}

impl Drop for Bmi088Reader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn sanity_check_bmi088() {
        // This test confirms that we can access the BMI088 code
        let accel_addr = 0x18; // Default BMI088 accelerometer address
        let gyro_addr = 0x68; // Default BMI088 gyroscope address

        assert_eq!(accel_addr, crate::ACCEL_ADDR);
        assert_eq!(gyro_addr, crate::GYRO_ADDR);

        println!("BMI088 crate version: {}", env!("CARGO_PKG_VERSION"));
    }
}
