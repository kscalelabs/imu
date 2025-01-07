mod registers;
use registers::{ChipRegisters, StatusRegisters, QuaternionRegisters, EulerRegisters, 
    LinearAccelRegisters, OperationMode, PowerMode, Constants};
use std::thread;
use std::time::Duration;
use byteorder::{ByteOrder, LittleEndian};
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
use log::{error, warn};

#[derive(Debug)]
pub enum Error {
    I2c(i2cdev::linux::LinuxI2CError),
    InvalidChipId,
    CalibrationFailed,
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
    pub fn new(i2c_bus: &str) -> Result<Self, Error> {
        let i2c = LinuxI2CDevice::new(i2c_bus, Constants::DefaultI2cAddr)?;
        let mut bno = Bno055 { i2c };
        
        // Verify we're talking to the right chip
        bno.verify_chip_id()?;
        
        // Reset the device
        bno.reset()?;
        
        // Configure for NDOF mode (9-axis fusion)
        bno.set_mode(OperationMode::Ndof)?;
        
        Ok(bno)
    }

    fn verify_chip_id(&mut self) -> Result<(), Error> {
        let chip_id = self.i2c.smbus_read_byte_data(ChipRegisters::ChipId as u8)?;
        if chip_id != Constants::ChipId {
            error!("Invalid chip ID. Expected 0xA0, got {:#x}", chip_id);
            return Err(Error::InvalidChipId);
        }
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        self.i2c.smbus_write_byte_data(StatusRegisters::SysTrigger as u8, 0x20)?;
        thread::sleep(Duration::from_millis(650));
        Ok(())
    }

    pub fn get_quaternion(&mut self) -> Result<Quaternion, Error> {
        let mut buf = [0u8; 8];
        
        // Read all quaternion data at once
        for i in 0..8 {
            buf[i] = self.i2c.smbus_read_byte_data(
                (QuaternionRegisters::WLsb as u8) + i
            )?;
        }

        let scale = 1.0 / ((1 << 14) as f32);
        Ok(Quaternion {
            w: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            x: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            y: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
            z: (LittleEndian::read_i16(&buf[6..8]) as f32) * scale,
        })
    }

    pub fn get_euler_angles(&mut self) -> Result<EulerAngles, Error> {
        let mut buf = [0u8; 6];
        
        // Read all euler angle data at once
        for i in 0..6 {
            buf[i] = self.i2c.smbus_read_byte_data(
                (EulerRegisters::HLsb as u8) + i
            )?;
        }
    
        // Convert to degrees (scale factor is 16)
        let scale = 1.0 / 16.0;
        Ok(EulerAngles {
            yaw: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            roll: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            pitch: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
        })
    }
    
    pub fn get_linear_acceleration(&mut self) -> Result<Vector3, Error> {
        let mut buf = [0u8; 6];
        
        // Read all linear acceleration data at once
        for i in 0..6 {
            buf[i] = self.i2c.smbus_read_byte_data(
                (LinearAccelRegisters::XLsb as u8) + i
            )?;
        }
    
        // Convert to m/s² (scale factor is 100)
        let scale = 1.0 / 100.0;
        Ok(Vector3 {
            x: (LittleEndian::read_i16(&buf[0..2]) as f32) * scale,
            y: (LittleEndian::read_i16(&buf[2..4]) as f32) * scale,
            z: (LittleEndian::read_i16(&buf[4..6]) as f32) * scale,
        })
    }
    
    pub fn set_mode(&mut self, mode: OperationMode) -> Result<(), Error> {
        self.i2c.smbus_write_byte_data(StatusRegisters::OprMode as u8, mode as u8)?;
        // Wait for mode switch to complete
        thread::sleep(Duration::from_millis(20));
        Ok(())
    }
}
