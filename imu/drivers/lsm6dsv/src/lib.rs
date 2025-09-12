// LSM6DSV 6-axis IMU driver for SPI communication

use byteorder::{ByteOrder, LittleEndian};
use imu_traits::{ImuData, ImuError, ImuReader, Vector3};
use log::{debug, error, warn};
use std::sync::{mpsc, Arc, RwLock};
use std::thread;
use std::time::Duration;

mod registers;
mod spi;

use registers::*;
use spi::Lsm6dsvSpi;

/// Low-level LSM6DSV driver
pub struct Lsm6dsv {
    spi: Lsm6dsvSpi,
    accel_range: AccelRange,
    gyro_range: GyroRange,
}

impl Lsm6dsv {
    /// Initialize the LSM6DSV sensor on the given SPI device
    pub fn new(spi_device: &str) -> Result<Self, ImuError> {
        debug!("Initializing LSM6DSV...");
        
        let mut spi = Lsm6dsvSpi::new(spi_device)?;
        
        // Verify chip ID
        let chip_id = spi.read_register(Register::WhoAmI as u8)?;
        if chip_id != Constants::WHO_AM_I_VALUE {
            return Err(ImuError::DeviceError(format!(
                "Invalid chip ID: expected 0x{:02X}, got 0x{:02X}",
                Constants::WHO_AM_I_VALUE, chip_id
            )));
        }
        debug!("LSM6DSV chip ID verified: 0x{:02X}", chip_id);
        
        // Soft reset
        spi.write_register(Register::Ctrl3C as u8, Constants::SOFT_RESET_CMD)?;
        thread::sleep(Duration::from_millis(100));
        
        // Configure accelerometer
        spi.write_register(Register::Ctrl1Xl as u8, 
            (AccelOdr::Hz104 as u8) | (AccelRange::G2 as u8))?;
        
        // Configure gyroscope
        spi.write_register(Register::Ctrl2G as u8, 
            (GyroOdr::Hz104 as u8) | (GyroRange::Dps250 as u8))?;
        
        // Enable accelerometer and gyroscope
        spi.write_register(Register::Ctrl4C as u8, 0x00)?; // Default settings
        spi.write_register(Register::Ctrl5C as u8, 0x00)?; // Default settings
        spi.write_register(Register::Ctrl6C as u8, 0x00)?; // Default settings
        spi.write_register(Register::Ctrl7G as u8, 0x00)?; // Default settings
        spi.write_register(Register::Ctrl8Xl as u8, 0x00)?; // Default settings
        spi.write_register(Register::Ctrl9Xl as u8, 0x00)?; // Default settings
        spi.write_register(Register::Ctrl10C as u8, 0x00)?; // Default settings
        
        // Wait for sensors to stabilize
        thread::sleep(Duration::from_millis(50));
        
        Ok(Lsm6dsv {
            spi,
            accel_range: AccelRange::G2,
            gyro_range: GyroRange::Dps250,
        })
    }
    
    /// Read raw accelerometer data
    pub fn read_raw_accelerometer(&mut self) -> Result<Vector3, ImuError> {
        let data = self.spi.read_registers(Register::OutXlA as u8, 6)?;
        
        let scale = match self.accel_range {
            AccelRange::G2 => Sensitivity::ACCEL_2G,
            AccelRange::G4 => Sensitivity::ACCEL_4G,
            AccelRange::G8 => Sensitivity::ACCEL_8G,
            AccelRange::G16 => Sensitivity::ACCEL_16G,
        };
        
        let raw_x = LittleEndian::read_i16(&data[0..2]) as f32;
        let raw_y = LittleEndian::read_i16(&data[2..4]) as f32;
        let raw_z = LittleEndian::read_i16(&data[4..6]) as f32;
        
        // Convert from mg to m/s² (multiply by 9.80665/1000)
        let g_to_ms2 = 9.80665 / 1000.0;
        Ok(Vector3 {
            x: raw_x * scale * g_to_ms2,
            y: raw_y * scale * g_to_ms2,
            z: raw_z * scale * g_to_ms2,
        })
    }
    
    /// Read raw gyroscope data
    pub fn read_raw_gyroscope(&mut self) -> Result<Vector3, ImuError> {
        let data = self.spi.read_registers(Register::OutXlG as u8, 6)?;
        
        let scale = match self.gyro_range {
            GyroRange::Dps125 => Sensitivity::GYRO_125,
            GyroRange::Dps250 => Sensitivity::GYRO_250,
            GyroRange::Dps500 => Sensitivity::GYRO_500,
            GyroRange::Dps1000 => Sensitivity::GYRO_1000,
            GyroRange::Dps2000 => Sensitivity::GYRO_2000,
            GyroRange::Dps4000 => Sensitivity::GYRO_4000,
        };
        
        let raw_x = LittleEndian::read_i16(&data[0..2]) as f32;
        let raw_y = LittleEndian::read_i16(&data[2..4]) as f32;
        let raw_z = LittleEndian::read_i16(&data[4..6]) as f32;
        
        // Convert from mdps to deg/s (divide by 1000)
        Ok(Vector3 {
            x: raw_x * scale / 1000.0,
            y: raw_y * scale / 1000.0,
            z: raw_z * scale / 1000.0,
        })
    }
    
    /// Read temperature in Celsius
    pub fn read_temperature(&mut self) -> Result<f32, ImuError> {
        let data = self.spi.read_registers(Register::OutTempL as u8, 2)?;
        let raw_temp = LittleEndian::read_i16(&data[0..2]) as f32;
        Ok(raw_temp * TEMP_SENSITIVITY + TEMP_OFFSET)
    }
    
    /// Check if new data is available
    pub fn is_data_ready(&mut self) -> Result<bool, ImuError> {
        let status = self.spi.read_register(Register::StatusReg as u8)?;
        Ok((status & 0x03) != 0) // Check if either accel or gyro data is ready
    }
}

/// LSM6DSVReader runs a background thread to update sensor data continuously
pub struct Lsm6dsvReader {
    data: Arc<RwLock<ImuData>>,
    command_tx: mpsc::Sender<Lsm6dsvCommand>,
    running: Arc<RwLock<bool>>,
}

/// Commands sent to the reading thread
#[derive(Debug)]
pub enum Lsm6dsvCommand {
    SetAccelRange(AccelRange),
    SetGyroRange(GyroRange),
    SetAccelOdr(AccelOdr),
    SetGyroOdr(GyroOdr),
    Reset,
    Stop,
}

impl Lsm6dsvReader {
    /// Create a new LSM6DSVReader
    pub fn new(spi_device: &str) -> Result<Self, ImuError> {
        debug!("Initializing LSM6DSV Reader...");
        let data = Arc::new(RwLock::new(ImuData::default()));
        let running = Arc::new(RwLock::new(true));
        let (command_tx, command_rx) = mpsc::channel();
        let reader = Lsm6dsvReader {
            data: Arc::clone(&data),
            command_tx,
            running: Arc::clone(&running),
        };
        reader.start_reading_thread(spi_device, command_rx)?;
        Ok(reader)
    }
    
    fn start_reading_thread(
        &self,
        spi_device: &str,
        command_rx: mpsc::Receiver<Lsm6dsvCommand>,
    ) -> Result<(), ImuError> {
        let data = Arc::clone(&self.data);
        let running = Arc::clone(&self.running);
        let spi_device = spi_device.to_string();
        let (tx, rx) = mpsc::channel();
        
        thread::spawn(move || {
            let init_result = Lsm6dsv::new(&spi_device);
            if let Err(e) = init_result {
                error!("Failed to initialize LSM6DSV: {}", e);
                let _ = tx.send(Err(e));
                return;
            }
            let mut imu = init_result.unwrap();
            let _ = tx.send(Ok(()));
            
            while let Ok(guard) = running.read() {
                if !*guard {
                    break;
                }
                
                // Handle incoming commands
                if let Ok(cmd) = command_rx.try_recv() {
                    match cmd {
                        Lsm6dsvCommand::SetAccelRange(range) => {
                            imu.accel_range = range;
                            // Update the control register
                            let range_val = range as u8;
                            let _ = imu.spi.read_register(Register::Ctrl1Xl as u8)
                                .and_then(|ctrl| {
                                    imu.spi.write_register(Register::Ctrl1Xl as u8, (ctrl & 0x0F) | range_val)
                                });
                        }
                        Lsm6dsvCommand::SetGyroRange(range) => {
                            imu.gyro_range = range;
                            // Update the control register
                            let range_val = range as u8;
                            let _ = imu.spi.read_register(Register::Ctrl2G as u8)
                                .and_then(|ctrl| {
                                    imu.spi.write_register(Register::Ctrl2G as u8, (ctrl & 0x0F) | range_val)
                                });
                        }
                        Lsm6dsvCommand::SetAccelOdr(odr) => {
                            // Update the control register
                            let odr_val = odr as u8;
                            let _ = imu.spi.read_register(Register::Ctrl1Xl as u8)
                                .and_then(|ctrl| {
                                    imu.spi.write_register(Register::Ctrl1Xl as u8, (ctrl & 0xF0) | odr_val)
                                });
                        }
                        Lsm6dsvCommand::SetGyroOdr(odr) => {
                            // Update the control register
                            let odr_val = odr as u8;
                            let _ = imu.spi.read_register(Register::Ctrl2G as u8)
                                .and_then(|ctrl| {
                                    imu.spi.write_register(Register::Ctrl2G as u8, (ctrl & 0xF0) | odr_val)
                                });
                        }
                        Lsm6dsvCommand::Reset => {
                            // Reinitialize if needed
                            imu = Lsm6dsv::new(&spi_device).unwrap();
                        }
                        Lsm6dsvCommand::Stop => {
                            if let Ok(mut w) = running.write() {
                                *w = false;
                            }
                            break;
                        }
                    }
                }
                
                // Read sensor data
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
                    sensor_data.temperature = Some(temp);
                }
                
                if let Ok(mut shared) = data.write() {
                    *shared = sensor_data;
                }
                
                thread::sleep(Duration::from_millis(10));
            }
        });
        
        rx.recv()?
    }
    
    /// Reset the LSM6DSV sensor
    pub fn reset(&self) -> Result<(), ImuError> {
        self.command_tx.send(Lsm6dsvCommand::Reset)?;
        Ok(())
    }
    
    /// Set accelerometer range
    pub fn set_accel_range(&self, range: AccelRange) -> Result<(), ImuError> {
        self.command_tx.send(Lsm6dsvCommand::SetAccelRange(range))?;
        Ok(())
    }
    
    /// Set gyroscope range
    pub fn set_gyro_range(&self, range: GyroRange) -> Result<(), ImuError> {
        self.command_tx.send(Lsm6dsvCommand::SetGyroRange(range))?;
        Ok(())
    }
    
    /// Set accelerometer output data rate
    pub fn set_accel_odr(&self, odr: AccelOdr) -> Result<(), ImuError> {
        self.command_tx.send(Lsm6dsvCommand::SetAccelOdr(odr))?;
        Ok(())
    }
    
    /// Set gyroscope output data rate
    pub fn set_gyro_odr(&self, odr: GyroOdr) -> Result<(), ImuError> {
        self.command_tx.send(Lsm6dsvCommand::SetGyroOdr(odr))?;
        Ok(())
    }
}

impl ImuReader for Lsm6dsvReader {
    /// Returns the most recent sensor data
    fn get_data(&self) -> Result<ImuData, ImuError> {
        Ok(self.data.read().map(|data| *data)?)
    }
    
    /// Stops the reading thread
    fn stop(&self) -> Result<(), ImuError> {
        self.command_tx.send(Lsm6dsvCommand::Stop)?;
        Ok(())
    }
}

impl Drop for Lsm6dsvReader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_register_values() {
        assert_eq!(Register::WhoAmI as u8, 0x0F);
        assert_eq!(Register::Ctrl1Xl as u8, 0x10);
        assert_eq!(Register::Ctrl2G as u8, 0x11);
        assert_eq!(Constants::WHO_AM_I_VALUE, 0x70);
    }
    
    #[test]
    fn test_sensitivity_values() {
        assert_eq!(Sensitivity::ACCEL_2G, 0.061);
        assert_eq!(Sensitivity::GYRO_250, 8.75);
    }
}

