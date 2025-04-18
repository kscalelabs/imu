use std::error::Error as StdError;
use std::fmt;

// --- Basic Types ---
#[derive(Debug, Clone, Copy, Default)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl fmt::Display for Vector3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vector3(x={}, y={}, z={})", self.x, self.y, self.z)
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl fmt::Display for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Quaternion(w={}, x={}, y={}, z={})", self.w, self.x, self.y, self.z)
    }
}

// --- Standard IMU Data ---
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    /// Acceleration including gravity (m/s²)
    pub accelerometer: Option<Vector3>,
    /// Angular velocity (deg/s)
    pub gyroscope: Option<Vector3>,
    /// Magnetic field vector (micro Tesla, µT)
    pub magnetometer: Option<Vector3>,
    /// Orientation as a unit quaternion (WXYZ order)
    pub quaternion: Option<Quaternion>,
    /// Orientation as Euler angles (deg)
    pub euler: Option<Vector3>,
    /// Linear acceleration (acceleration without gravity) (m/s²)
    pub linear_acceleration: Option<Vector3>,
    /// Estimated gravity vector (m/s²)
    pub gravity: Option<Vector3>,
    /// Temperature (°C)
    pub temperature: Option<f32>,
}

// --- Standard Error Type ---
#[derive(Debug)]
pub enum ImuError {
    /// Error originating from the underlying device communication (I2C, Serial, CAN)
    DeviceError(String),
    /// Error reading data from the device or internal state
    ReadError(String),
    /// Error writing commands or configuration to the device
    WriteError(String),
    /// Error during device configuration or setup
    ConfigurationError(String),
    /// Error related to multithreading locks (e.g., poisoned)
    LockError(String),
    /// Error sending a command to the reader thread
    CommandSendError(String),
    /// Functionality not supported by this specific IMU implementation
    NotSupported(String),
    /// Catch-all for other errors
    Other(String),
}

impl fmt::Display for ImuError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ImuError::DeviceError(s) => write!(f, "Device error: {}", s),
            ImuError::ReadError(s) => write!(f, "Read error: {}", s),
            ImuError::WriteError(s) => write!(f, "Write error: {}", s),
            ImuError::ConfigurationError(s) => write!(f, "Configuration error: {}", s),
            ImuError::LockError(s) => write!(f, "Lock error: {}", s),
            ImuError::CommandSendError(s) => write!(f, "Command send error: {}", s),
            ImuError::NotSupported(s) => write!(f, "Not supported: {}", s),
            ImuError::Other(s) => write!(f, "Other IMU error: {}", s),
        }
    }
}

impl StdError for ImuError {}
pub trait ImuReader {
    /// Retrieves the latest available IMU data.
    fn get_data(&self) -> Result<ImuData, ImuError>;

    fn stop(&self) -> Result<(), ImuError>;
}

// --- Re-export concrete reader types based on features ---

#[cfg(feature = "bno055")]
pub use bno055::Bno055Reader;

#[cfg(feature = "bmi088")]
pub use bmi088::Bmi088Reader;

#[cfg(feature = "hiwonder")]
pub use hiwonder::HiwonderReader;
#[cfg(feature = "hiwonder")]
pub use hiwonder::ImuFrequency as HiwonderImuFrequency;

#[cfg(feature = "hexmove")]
pub use hexmove::ImuReader as HexmoveImuReader;
