// LSM6DSV Register definitions and constants

/// LSM6DSV Register addresses
#[repr(u8)]
pub enum Register {
    // WHO_AM_I register
    WhoAmI = 0x0F,
    
    // Control registers
    Ctrl1Xl = 0x10,  // Accelerometer control
    Ctrl2G = 0x11,   // Gyroscope control
    Ctrl3C = 0x12,   // Control register 3
    Ctrl4C = 0x13,   // Control register 4
    Ctrl5C = 0x14,   // Control register 5
    Ctrl6C = 0x15,   // Control register 6
    Ctrl7G = 0x16,   // Gyroscope control 2
    Ctrl8Xl = 0x17,  // Accelerometer control 2
    Ctrl9Xl = 0x18,  // Accelerometer control 3
    Ctrl10C = 0x19,  // Control register 10
    
    // Status registers
    StatusReg = 0x1E,  // Status register
    
    // Temperature register
    OutTempL = 0x20,  // Temperature output LSB
    OutTempH = 0x21,  // Temperature output MSB
    
    // Gyroscope output registers
    OutXlG = 0x22,    // Gyroscope X-axis LSB
    OutXhG = 0x23,    // Gyroscope X-axis MSB
    OutYlG = 0x24,    // Gyroscope Y-axis LSB
    OutYhG = 0x25,    // Gyroscope Y-axis MSB
    OutZlG = 0x26,    // Gyroscope Z-axis LSB
    OutZhG = 0x27,    // Gyroscope Z-axis MSB
    
    // Accelerometer output registers
    OutXlA = 0x28,    // Accelerometer X-axis LSB
    OutXhA = 0x29,    // Accelerometer X-axis MSB
    OutYlA = 0x2A,    // Accelerometer Y-axis LSB
    OutYhA = 0x2B,    // Accelerometer Y-axis MSB
    OutZlA = 0x2C,    // Accelerometer Z-axis LSB
    OutZhA = 0x2D,    // Accelerometer Z-axis MSB
}

/// LSM6DSV Constants
pub mod Constants {
    /// Expected WHO_AM_I value for LSM6DSV
    pub const WHO_AM_I_VALUE: u8 = 0x70;
    
    /// Soft reset command
    pub const SOFT_RESET_CMD: u8 = 0x01;
    
    /// Power-on command for accelerometer
    pub const ACCEL_POWER_ON: u8 = 0x04;
    
    /// Power-on command for gyroscope
    pub const GYRO_POWER_ON: u8 = 0x04;
}

/// Accelerometer output data rate (ODR) settings
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum AccelOdr {
    Off = 0x00,
    Hz12_5 = 0x01,
    Hz26 = 0x02,
    Hz52 = 0x03,
    Hz104 = 0x04,
    Hz208 = 0x05,
    Hz416 = 0x06,
    Hz833 = 0x07,
    Hz1660 = 0x08,
    Hz3330 = 0x09,
    Hz6660 = 0x0A,
}

/// Accelerometer full-scale range settings
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum AccelRange {
    G2 = 0x00,   // ±2g
    G4 = 0x02,   // ±4g
    G8 = 0x03,   // ±8g
    G16 = 0x01,  // ±16g
}

/// Gyroscope output data rate (ODR) settings
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum GyroOdr {
    Off = 0x00,
    Hz12_5 = 0x01,
    Hz26 = 0x02,
    Hz52 = 0x03,
    Hz104 = 0x04,
    Hz208 = 0x05,
    Hz416 = 0x06,
    Hz833 = 0x07,
    Hz1660 = 0x08,
    Hz3330 = 0x09,
    Hz6660 = 0x0A,
}

/// Gyroscope full-scale range settings
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum GyroRange {
    Dps125 = 0x00,   // ±125 dps
    Dps250 = 0x01,   // ±250 dps
    Dps500 = 0x02,   // ±500 dps
    Dps1000 = 0x03,  // ±1000 dps
    Dps2000 = 0x04,  // ±2000 dps
    Dps4000 = 0x05,  // ±4000 dps
}

/// Sensitivity values for different ranges
pub mod Sensitivity {
    // Accelerometer sensitivity (mg/LSB)
    pub const ACCEL_2G: f32 = 0.061;   // ±2g
    pub const ACCEL_4G: f32 = 0.122;   // ±4g
    pub const ACCEL_8G: f32 = 0.244;   // ±8g
    pub const ACCEL_16G: f32 = 0.488;  // ±16g
    
    // Gyroscope sensitivity (mdps/LSB)
    pub const GYRO_125: f32 = 4.375;   // ±125 dps
    pub const GYRO_250: f32 = 8.75;    // ±250 dps
    pub const GYRO_500: f32 = 17.50;   // ±500 dps
    pub const GYRO_1000: f32 = 35.0;   // ±1000 dps
    pub const GYRO_2000: f32 = 70.0;   // ±2000 dps
    pub const GYRO_4000: f32 = 140.0;  // ±4000 dps
}

/// Temperature sensitivity (degC/LSB)
pub const TEMP_SENSITIVITY: f32 = 1.0 / 256.0;
pub const TEMP_OFFSET: f32 = 25.0;

