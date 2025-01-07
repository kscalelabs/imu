use num_derive::{FromPrimitive, ToPrimitive};

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum ChipRegisters {
    ChipId = 0x00,
    AccId = 0x01,
    MagId = 0x02,
    GyrId = 0x03,
    SwRevIdLsb = 0x04,
    SwRevIdMsb = 0x05,
    BlRevId = 0x06,
    PageId = 0x07,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum AccelRegisters {
    XLsb = 0x08,
    XMsb = 0x09,
    YLsb = 0x0A,
    YMsb = 0x0B,
    ZLsb = 0x0C,
    ZMsb = 0x0D,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum MagRegisters {
    XLsb = 0x0E,
    XMsb = 0x0F,
    YLsb = 0x10,
    YMsb = 0x11,
    ZLsb = 0x12,
    ZMsb = 0x13,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum GyroRegisters {
    XLsb = 0x14,
    XMsb = 0x15,
    YLsb = 0x16,
    YMsb = 0x17,
    ZLsb = 0x18,
    ZMsb = 0x19,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum EulerRegisters {
    HLsb = 0x1A,
    HMsb = 0x1B,
    RLsb = 0x1C,
    RMsb = 0x1D,
    PLsb = 0x1E,
    PMsb = 0x1F,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum QuaternionRegisters {
    WLsb = 0x20,
    WMsb = 0x21,
    XLsb = 0x22,
    XMsb = 0x23,
    YLsb = 0x24,
    YMsb = 0x25,
    ZLsb = 0x26,
    ZMsb = 0x27,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum LinearAccelRegisters {
    XLsb = 0x28,
    XMsb = 0x29,
    YLsb = 0x2A,
    YMsb = 0x2B,
    ZLsb = 0x2C,
    ZMsb = 0x2D,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum StatusRegisters {
    Temperature = 0x34,
    CalibStat = 0x35,
    StResult = 0x36,
    IntSta = 0x37,
    SysClkStatus = 0x38,
    SysStatus = 0x39,
    SysErr = 0x3A,
    UnitSel = 0x3B,
    OprMode = 0x3D,
    PwrMode = 0x3E,
    SysTrigger = 0x3F,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum PowerMode {
    Normal = 0x00,
    Low = 0x01,
    Suspend = 0x02,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum OperationMode {
    Config = 0x00,
    AccOnly = 0x01,
    MagOnly = 0x02,
    GyrOnly = 0x03,
    AccMag = 0x04,
    AccGyro = 0x05,
    MagGyro = 0x06,
    Amg = 0x07,
    Imu = 0x08,
    Compass = 0x09,
    M4g = 0x0A,
    NdofFmcOff = 0x0B,
    Ndof = 0x0C,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum Constants {
    ChipId = 0xA0,
    DefaultI2cAddr = 0x28,
}
