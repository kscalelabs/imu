use bitflags::bitflags;
use hiwonder_macros::{BytableRegistrableCommand, DefaultableCommand, Registrable};
use imu_traits::{ImuError, ImuFrequency};
use serde::{Deserialize, Serialize};
use strum_macros::EnumIter;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumIter, Serialize, Deserialize)]
pub enum Register {
    Save = 0x00,
    CalSw = 0x01,        // Calibration mode
    Rsw = 0x02,          // output content
    Rrate = 0x03,        // output rate
    Baud = 0x04,         // Serial port baud rate
    AxOffset = 0x05,     // Acceleration X Bias
    AyOffset = 0x06,     // Acceleration Y Bias
    AzOffset = 0x07,     // Acceleration Z Bias
    GxOffset = 0x08,     // Angular velocity X Bias
    GyOffset = 0x09,     // Angular velocity Y Bias
    GzOffset = 0x0A,     // Angular velocity Z Bias
    HxOffset = 0x0B,     // Magnetic Field X Bias
    HyOffset = 0x0C,     // Magnetic Field Y Bias
    HzOffset = 0x0D,     // Magnetic Field Z Bias
    D0Mode = 0x0E,       // D0 Pin mode
    D1Mode = 0x0F,       // D1 Pin mode
    D2Mode = 0x10,       // D2 Pin mode
    D3Mode = 0x11,       // D3 Pin mode
    IicAddr = 0x1A,      // Device address
    LedOff = 0x1B,       // Turn off the LED lights
    MagRangX = 0x1C,     // Magnetic Field X Calibration Range
    MagRangY = 0x1D,     // Magnetic Field Y Calibration Range
    MagRangZ = 0x1E,     // Magnetic Field Z Calibration Range
    Bandwidth = 0x1F,    // Bandwidth
    GyroRange = 0x20,    // Gyroscope range
    AccRange = 0x21,     // Acceleration range
    Sleep = 0x22,        // Hibernate
    Orient = 0x23,       // Installation direction
    Axis6 = 0x24,        // algorithm
    FilTk = 0x25,        // Dynamic filtering
    GpsBaud = 0x26,      // GPS baud rate
    ReadAddr = 0x27,     // read register
    AccFilt = 0x2A,      // acceleration filter
    PowOnSend = 0x2D,    // command start
    Version = 0x2E,      // version number (Read-only)
    YYMM = 0x30,         // Year Month
    DDHH = 0x31,         // Day Hour
    MMSS = 0x32,         // Minute Second
    Ms = 0x33,           // Millisecond
    Ax = 0x34,           // Acceleration X (Read-only)
    Ay = 0x35,           // Acceleration Y (Read-only)
    Az = 0x36,           // Acceleration Z (Read-only)
    Gx = 0x37,           // Angular velocity X (Read-only)
    Gy = 0x38,           // Angular velocity Y (Read-only)
    Gz = 0x39,           // Angular velocity Z (Read-only)
    Hx = 0x3A,           // Magnetic Field X (Read-only)
    Hy = 0x3B,           // Magnetic Field Y (Read-only)
    Hz = 0x3C,           // Magnetic Field Z (Read-only)
    Roll = 0x3D,         // roll angle (Read-only)
    Pitch = 0x3E,        // Pitch angle (Read-only)
    Yaw = 0x3F,          // Heading (Read-only)
    Temp = 0x40,         // temperature (Read-only)
    D0Status = 0x41,     // D0 pin state (Read-only)
    D1Status = 0x42,     // D1 pin state (Read-only)
    D2Status = 0x43,     // D2 pin state (Read-only)
    D3Status = 0x44,     // D3 pin state (Read-only)
    PressureL = 0x45,    // Air pressure low 16 bits (Read-only)
    PressureH = 0x46,    // Air pressure high 16 bits (Read-only)
    HeightL = 0x47,      // Height lower 16 bits (Read-only)
    HeightH = 0x48,      // Height high 16 bits (Read-only)
    LonL = 0x49,         // Longitude lower 16 bits (Read-only)
    LonH = 0x4A,         // Longitude high 16 bits (Read-only)
    LatL = 0x4B,         // Latitude lower 16 bits (Read-only)
    LatH = 0x4C,         // Latitude high 16 bits (Read-only)
    GpsHeight = 0x4D,    // GPS Altitude (Read-only)
    GpsYaw = 0x4E,       // GPS heading angle (Read-only)
    GpsVL = 0x4F,        // GPS ground speed low 16 bits (Read-only)
    GpsVH = 0x50,        // GPS ground speed high 16 bits (Read-only)
    Q0 = 0x51,           // Quaternion 0 (Read-only)
    Q1 = 0x52,           // Quaternion 1 (Read-only)
    Q2 = 0x53,           // Quaternion 2 (Read-only)
    Q3 = 0x54,           // Quaternion 3 (Read-only)
    SvNum = 0x55,        // number of satellites (Read-only)
    Pdop = 0x56,         // Position accuracy (Read-only)
    Hdop = 0x57,         // Horizontal accuracy (Read-only)
    Vdop = 0x58,         // vertical accuracy (Read-only)
    DelayT = 0x59,       // Alarm signal delay
    XMin = 0x5A,         // X-axis angle alarm minimum value
    XMax = 0x5B,         // X-axis angle alarm maximum value
    BatVal = 0x5C,       // Supply voltage (Read-only)
    AlarmPin = 0x5D,     // Alarm Pin Mapping
    YMin = 0x5E,         // Y-axis angle alarm minimum value
    YMax = 0x5F,         // Y-axis angle alarm maximum value
    GyroCaliThr = 0x61,  // Gyro Still Threshold
    AlarmLevel = 0x62,   // Angle alarm level
    GyroCaliTime = 0x63, // Gyro auto calibration time
    TrigTime = 0x68,     // Alarm continuous trigger time
    Key = 0x69,          // unlock
    WError = 0x6A,       // Gyroscope change value (Read-only)
    TimeZone = 0x6B,     // GPS time zone
    WzTime = 0x6E,       // Angular velocity continuous rest time
    WzStatic = 0x6F,     // Angular velocity integral threshold
    ModDelay = 0x74,     // 485 data response delay
    XRefRoll = 0x79,     // Roll angle zero reference value (Read-only)
    YRefPitch = 0x7A,    // Pitch angle zero reference value (Read-only)
    NumberId1 = 0x7F,    // Device ID 1-2 (Read-only)
    NumberId2 = 0x80,    // Device ID 3-4 (Read-only)
    NumberId3 = 0x81,    // Device ID 5-6 (Read-only)
    NumberId4 = 0x82,    // Device ID 7-8 (Read-only)
    NumberId5 = 0x83,    // Device ID 9-10 (Read-only)
    NumberId6 = 0x84,    // Device ID 11-12 (Read-only)
}

pub trait Bytable {
    fn to_bytes(&self) -> Vec<u8>;
}

pub trait Registrable {
    fn register(&self) -> Register;
}

pub trait BytableRegistrable: Bytable + Registrable {}

#[derive(Registrable)]
pub struct Command {
    pub register: Register,
    pub data: [u8; 2],
}

impl Command {
    pub fn new(register: Register, data: [u8; 2]) -> Self {
        Self { register, data }
    }
}

impl Bytable for Command {
    fn to_bytes(&self) -> Vec<u8> {
        vec![0xFF, 0xAA, self.register as u8, self.data[0], self.data[1]]
    }
}

impl Default for Command {
    fn default() -> Self {
        Self {
            register: Register::Save,
            data: [0x00, 0x00],
        }
    }
}

// Common commands
#[derive(BytableRegistrableCommand, DefaultableCommand)]
pub struct UnlockCommand {
    pub command: Command,
}

impl UnlockCommand {
    pub fn new() -> Self {
        let command = Command::new(Register::Key, [0x88, 0xB5]);
        Self { command }
    }
}

#[derive(BytableRegistrableCommand)]
pub struct ReadAddressCommand {
    pub command: Command,
}

impl ReadAddressCommand {
    pub fn new(register: Register) -> Self {
        let command = Command::new(Register::ReadAddr, [register as u8, 0x00]);
        Self { command }
    }
}

impl Default for ReadAddressCommand {
    fn default() -> Self {
        Self::new(Register::IicAddr)
    }
}

#[derive(BytableRegistrableCommand)]
pub struct FusionAlgorithmCommand {
    pub command: Command,
}

pub enum FusionAlgorithm {
    NineAxis,
    SixAxis,
}

impl FusionAlgorithmCommand {
    pub fn new(algorithm: FusionAlgorithm) -> Self {
        let data: [u8; 2] = match algorithm {
            FusionAlgorithm::NineAxis => [0x00, 0x00],
            FusionAlgorithm::SixAxis => [0x01, 0x00],
        };
        let command = Command::new(Register::Axis6, data);
        Self { command }
    }
}

impl Default for FusionAlgorithmCommand {
    fn default() -> Self {
        Self::new(FusionAlgorithm::NineAxis)
    }
}

#[derive(BytableRegistrableCommand)]
pub struct EnableOutputCommand {
    pub command: Command,
}

// Note that the Hiwonder imu will reject an empty flag set.
bitflags! {
    #[derive(Debug, Copy, Clone)]
    pub struct Output: u16 {
        const TIME = 1 << 0; // 0x50
        const ACC = 1 << 1; // 0x51
        const GYRO = 1 << 2; // 0x52
        const ANGLE = 1 << 3; // 0x53
        const MAG = 1 << 4; // 0x54
        const PORT = 1 << 5; // 0x55
        const PRESS = 1 << 6; // 0x56
        const GPS = 1 << 7; // 0x57
        const VELOCITY = 1 << 8; // 0x58
        const QUATERNION = 1 << 9; // 0x59
        const GPS_ACCURACY = 1 << 10; // 0x5A
    }
}

impl EnableOutputCommand {
    pub fn new(output: Output) -> Self {
        let bits = output.bits();
        let data = [bits as u8, (bits >> 8) as u8];
        let command = Command::new(Register::Rsw, data);
        Self { command }
    }
}

impl Default for EnableOutputCommand {
    fn default() -> Self {
        Self::new(Output::ACC | Output::GYRO | Output::ANGLE | Output::QUATERNION)
    }
}

#[derive(BytableRegistrableCommand, DefaultableCommand)]
pub struct SaveCommand {
    pub command: Command,
}

impl SaveCommand {
    pub fn new() -> Self {
        let command = Command::new(Register::Save, [0x00, 0x00]);
        Self { command }
    }
}

#[derive(BytableRegistrableCommand, DefaultableCommand)]
pub struct RebootCommand {
    pub command: Command,
}

impl RebootCommand {
    pub fn new() -> Self {
        let command = Command::new(Register::Save, [0xFF, 0x00]);
        Self { command }
    }
}

#[derive(BytableRegistrableCommand, DefaultableCommand)]
pub struct FactoryResetCommand {
    pub command: Command,
}

impl FactoryResetCommand {
    pub fn new() -> Self {
        let command = Command::new(Register::Save, [0x01, 0x00]);
        Self { command }
    }
}

impl Bytable for ImuFrequency {
    fn to_bytes(&self) -> Vec<u8> {
        match self {
            ImuFrequency::Hz0_2 => vec![0x01, 0x00],
            ImuFrequency::Hz0_5 => vec![0x02, 0x00],
            ImuFrequency::Hz1 => vec![0x03, 0x00],
            ImuFrequency::Hz2 => vec![0x04, 0x00],
            ImuFrequency::Hz5 => vec![0x05, 0x00],
            ImuFrequency::Hz10 => vec![0x06, 0x00],
            ImuFrequency::Hz20 => vec![0x07, 0x00],
            ImuFrequency::Hz50 => vec![0x08, 0x00],
            ImuFrequency::Hz100 => vec![0x09, 0x00],
            ImuFrequency::Hz200 => vec![0x0B, 0x00],
            ImuFrequency::Single => vec![0x0C, 0x00],
            ImuFrequency::None => vec![0x0D, 0x00],
        }
    }
}

#[derive(BytableRegistrableCommand)]
pub struct SetFrequencyCommand {
    pub command: Command,
}

impl SetFrequencyCommand {
    pub fn new(frequency: ImuFrequency) -> Self {
        let data = frequency.to_bytes();
        let command = Command::new(Register::Rrate, [data[0], data[1]]);
        Self { command }
    }
}

impl Default for SetFrequencyCommand {
    fn default() -> Self {
        Self::new(ImuFrequency::Hz100)
    }
}

#[derive(BytableRegistrableCommand)]
pub struct SetBaudRateCommand {
    pub command: Command,
}

#[derive(Debug, Clone)]
pub enum BaudRate {
    Baud4800,
    Baud9600,
    Baud19200,
    Baud38400,
    Baud57600,
    Baud115200,
    Baud230400,
    Baud460800, // Only supported by WT931/JY931/HWT606/HWT906
    Baud921600, // Only supported by WT931/JY931/HWT606/HWT906
}

impl TryFrom<u32> for BaudRate {
    type Error = ImuError;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Ok(match value {
            4800 => BaudRate::Baud4800,
            9600 => BaudRate::Baud9600,
            19200 => BaudRate::Baud19200,
            38400 => BaudRate::Baud38400,
            57600 => BaudRate::Baud57600,
            115200 => BaudRate::Baud115200,
            230400 => BaudRate::Baud230400,
            460800 => BaudRate::Baud460800,
            921600 => BaudRate::Baud921600,
            _ => {
                return Err(ImuError::ConfigurationError(format!(
                    "Invalid baud rate: {}",
                    value
                )))
            }
        })
    }
}

impl TryInto<u32> for BaudRate {
    type Error = ImuError;

    fn try_into(self) -> Result<u32, Self::Error> {
        Ok(match self {
            BaudRate::Baud4800 => 4800,
            BaudRate::Baud9600 => 9600,
            BaudRate::Baud19200 => 19200,
            BaudRate::Baud38400 => 38400,
            BaudRate::Baud57600 => 57600,
            BaudRate::Baud115200 => 115200,
            BaudRate::Baud230400 => 230400,
            BaudRate::Baud460800 => 460800,
            BaudRate::Baud921600 => 921600,
        })
    }
}

impl Bytable for BaudRate {
    fn to_bytes(&self) -> Vec<u8> {
        match self {
            BaudRate::Baud4800 => vec![0x01, 0x00],
            BaudRate::Baud9600 => vec![0x02, 0x00],
            BaudRate::Baud19200 => vec![0x03, 0x00],
            BaudRate::Baud38400 => vec![0x04, 0x00],
            BaudRate::Baud57600 => vec![0x05, 0x00],
            BaudRate::Baud115200 => vec![0x06, 0x00],
            BaudRate::Baud230400 => vec![0x07, 0x00],
            BaudRate::Baud460800 => vec![0x08, 0x00],
            BaudRate::Baud921600 => vec![0x09, 0x00],
        }
    }
}

impl SetBaudRateCommand {
    pub fn new(baud_rate: BaudRate) -> Self {
        let data = baud_rate.to_bytes();
        let command = Command::new(Register::Baud, [data[0], data[1]]);
        Self { command }
    }
}

impl Default for SetBaudRateCommand {
    fn default() -> Self {
        Self::new(BaudRate::Baud230400)
    }
}

/// Represents the configurable bandwidth settings of the IMU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Bandwidth {
    Hz256 = 0x00,
    Hz188 = 0x01,
    Hz98 = 0x02,
    Hz42 = 0x03,
    Hz20 = 0x04,
    Hz10 = 0x05,
    Hz5 = 0x06,
}

impl Bytable for Bandwidth {
    fn to_bytes(&self) -> Vec<u8> {
        vec![*self as u8, 0x00]
    }
}

impl TryFrom<u32> for Bandwidth {
    type Error = ImuError;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Ok(match value {
            256 => Bandwidth::Hz256,
            188 => Bandwidth::Hz188,
            98 => Bandwidth::Hz98,
            42 => Bandwidth::Hz42,
            20 => Bandwidth::Hz20,
            10 => Bandwidth::Hz10,
            5 => Bandwidth::Hz5,
            _ => {
                return Err(ImuError::ConfigurationError(format!(
                    "Invalid bandwidth: {}",
                    value
                )))
            }
        })
    }
}

#[derive(BytableRegistrableCommand)]
pub struct SetBandwidthCommand {
    pub command: Command,
}

impl SetBandwidthCommand {
    pub fn new(bandwidth: Bandwidth) -> Self {
        let data = bandwidth.to_bytes();
        let command = Command::new(Register::Bandwidth, [data[0], data[1]]);
        Self { command }
    }
}

impl Default for SetBandwidthCommand {
    fn default() -> Self {
        Self::new(Bandwidth::Hz42)
    }
}
