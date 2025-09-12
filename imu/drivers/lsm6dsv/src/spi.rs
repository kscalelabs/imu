// SPI communication layer for LSM6DSV

use spidev::{Spidev, SpidevOptions, SpidevTransfer, SpiModeFlags};
use imu_traits::ImuError;

/// SPI communication wrapper for LSM6DSV
pub struct Lsm6dsvSpi {
    spi: Spidev,
}

impl Lsm6dsvSpi {
    /// Initialize SPI communication with LSM6DSV
    pub fn new(spi_device: &str) -> Result<Self, ImuError> {
        let mut spi = Spidev::open(spi_device)
            .map_err(|e| ImuError::DeviceError(format!("Failed to open SPI device {}: {}", spi_device, e)))?;
        
        // Configure SPI mode and settings
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(10_000_000) // 10 MHz max speed for LSM6DSV
            .mode(SpiModeFlags::SPI_MODE_0) // CPOL=0, CPHA=0
            .build();
        
        spi.configure(&options)
            .map_err(|e| ImuError::DeviceError(format!("Failed to configure SPI: {}", e)))?;
        
        Ok(Lsm6dsvSpi { spi })
    }
    
    /// Write a single byte to a register
    pub fn write_register(&mut self, register: u8, value: u8) -> Result<(), ImuError> {
        let tx_buf = [register & 0x7F, value]; // Clear MSB for write operation
        let mut rx_buf = [0u8; 2];
        
        let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        self.spi.transfer(&mut transfer)
            .map_err(|e| ImuError::WriteError(format!("Failed to write register 0x{:02X}: {}", register, e)))?;
        
        Ok(())
    }
    
    /// Read a single byte from a register
    pub fn read_register(&mut self, register: u8) -> Result<u8, ImuError> {
        let tx_buf = [register | 0x80, 0x00]; // Set MSB for read operation
        let mut rx_buf = [0u8; 2];
        
        let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        self.spi.transfer(&mut transfer)
            .map_err(|e| ImuError::ReadError(format!("Failed to read register 0x{:02X}: {}", register, e)))?;
        
        Ok(rx_buf[1])
    }
    
    /// Read multiple bytes from consecutive registers
    pub fn read_registers(&mut self, start_register: u8, count: usize) -> Result<Vec<u8>, ImuError> {
        let mut tx_buf = vec![0u8; count + 1];
        let mut rx_buf = vec![0u8; count + 1];
        
        tx_buf[0] = start_register | 0x80; // Set MSB for read operation
        
        let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        self.spi.transfer(&mut transfer)
            .map_err(|e| ImuError::ReadError(format!("Failed to read registers starting at 0x{:02X}: {}", start_register, e)))?;
        
        Ok(rx_buf[1..].to_vec())
    }
    
    /// Write multiple bytes to consecutive registers
    pub fn write_registers(&mut self, start_register: u8, values: &[u8]) -> Result<(), ImuError> {
        let mut tx_buf = vec![0u8; values.len() + 1];
        let mut rx_buf = vec![0u8; values.len() + 1];
        
        tx_buf[0] = start_register & 0x7F; // Clear MSB for write operation
        tx_buf[1..].copy_from_slice(values);
        
        let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        self.spi.transfer(&mut transfer)
            .map_err(|e| ImuError::WriteError(format!("Failed to write registers starting at 0x{:02X}: {}", start_register, e)))?;
        
        Ok(())
    }
}

