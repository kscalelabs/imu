use std::error::Error as StdError;
use std::fmt;
use std::io;
use std::sync::mpsc;

// --- Basic Types ---
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn euler_to_quaternion(&self) -> Quaternion {
        // Convert Euler angles (in radians) to quaternion
        // Using the ZYX rotation order (yaw, pitch, roll)
        let (roll, pitch, yaw) = (self.x, self.y, self.z);

        let cr = (roll * 0.5).cos();
        let sr = (roll * 0.5).sin();
        let cp = (pitch * 0.5).cos();
        let sp = (pitch * 0.5).sin();
        let cy = (yaw * 0.5).cos();
        let sy = (yaw * 0.5).sin();

        let w = cr * cp * cy + sr * sp * sy;
        let x = sr * cp * cy - cr * sp * sy;
        let y = cr * sp * cy + sr * cp * sy;
        let z = cr * cp * sy - sr * sp * cy;

        Quaternion { w, x, y, z }
    }
}

impl fmt::Display for Vector3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vector3(x={}, y={}, z={})", self.x, self.y, self.z)
    }
}

impl Quaternion {
    pub fn rotate(&self, vector: Vector3) -> Vector3 {
        // Rotate a vector by a quaternion using the formula:
        // v' = q * v * q^-1
        // Where q^-1 is the conjugate since we assume unit quaternions
        let qw = self.w;
        let qx = self.x;
        let qy = self.y;
        let qz = self.z;
        let vx = vector.x;
        let vy = vector.y;
        let vz = vector.z;

        // Calculate rotation using quaternion multiplication
        let x = (1.0 - 2.0 * qy * qy - 2.0 * qz * qz) * vx
            + (2.0 * qx * qy - 2.0 * qz * qw) * vy
            + (2.0 * qx * qz + 2.0 * qy * qw) * vz;
        let y = (2.0 * qx * qy + 2.0 * qz * qw) * vx
            + (1.0 - 2.0 * qx * qx - 2.0 * qz * qz) * vy
            + (2.0 * qy * qz - 2.0 * qx * qw) * vz;
        let z = (2.0 * qx * qz - 2.0 * qy * qw) * vx
            + (2.0 * qy * qz + 2.0 * qx * qw) * vy
            + (1.0 - 2.0 * qx * qx - 2.0 * qy * qy) * vz;

        Vector3 { x, y, z }
    }
}

impl fmt::Display for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Quaternion(w={}, x={}, y={}, z={})",
            self.w, self.x, self.y, self.z
        )
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
    /// Calibration status
    pub calibration_status: Option<u8>,
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
    /// Invalid packet received from the device
    InvalidPacket(String),
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
            ImuError::InvalidPacket(s) => write!(f, "Invalid packet: {}", s),
            ImuError::Other(s) => write!(f, "Other IMU error: {}", s),
        }
    }
}

impl StdError for ImuError {}

// Add From implementations for common error types
impl From<io::Error> for ImuError {
    fn from(err: io::Error) -> Self {
        ImuError::DeviceError(err.to_string())
    }
}

impl From<serialport::Error> for ImuError {
    fn from(err: serialport::Error) -> Self {
        ImuError::DeviceError(err.to_string())
    }
}

impl<T> From<std::sync::PoisonError<T>> for ImuError {
    fn from(err: std::sync::PoisonError<T>) -> Self {
        ImuError::LockError(err.to_string())
    }
}

impl<T> From<mpsc::SendError<T>> for ImuError {
    fn from(err: mpsc::SendError<T>) -> Self {
        ImuError::CommandSendError(err.to_string())
    }
}

impl From<mpsc::RecvError> for ImuError {
    fn from(err: mpsc::RecvError) -> Self {
        ImuError::CommandSendError(err.to_string())
    }
}

pub trait ImuReader {
    /// Retrieves the latest available IMU data.
    fn get_data(&self) -> Result<ImuData, ImuError>;

    fn stop(&self) -> Result<(), ImuError>;
}

#[derive(Debug, Clone, Copy)]
pub enum ImuFrequency {
    Hz0_2,  // 0.2 Hz
    Hz0_5,  // 0.5 Hz
    Hz1,    // 1 Hz
    Hz2,    // 2 Hz
    Hz5,    // 5 Hz
    Hz10,   // 10 Hz
    Hz20,   // 20 Hz
    Hz50,   // 50 Hz
    Hz100,  // 100 Hz
    Hz200,  // 200 Hz
    Single, // Single reading
    None,   // No readings
}
