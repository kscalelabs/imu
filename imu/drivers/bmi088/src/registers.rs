use num_derive::{FromPrimitive, ToPrimitive};

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum AccelRegisters {
    ChipId = 0x00,
    Error = 0x02,
    Status = 0x03,
    AccelXLsb = 0x12,
    AccelXMsb = 0x13,
    AccelYLsb = 0x14,
    AccelYMsb = 0x15,
    AccelZLsb = 0x16,
    AccelZMsb = 0x17,
    TempMsb = 0x22,
    TempLsb = 0x23,
    PowerCtrl = 0x7D,
    PowerConf = 0x7C,
    AccConf = 0x40,
    AccRange = 0x41,
    SoftReset = 0x7E,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum GyroRegisters {
    ChipId = 0x00,
    XLsb = 0x02,
    XMsb = 0x03,
    YLsb = 0x04,
    YMsb = 0x05,
    ZLsb = 0x06,
    ZMsb = 0x07,
    Range = 0x0F,
    Bandwidth = 0x10,
    PowerMode = 0x11,
}

#[derive(Debug, Clone, Copy)]
pub enum Constants {
    AccelChipIdValue = 0x1E,
    AccelI2cAddr = 0x19, // SDO pulled high
    GyroI2cAddr = 0x69,  // SDO pulled high
    SoftResetCmd = 0xB6,
}

#[derive(Debug, Clone, Copy)]
pub enum AccelRange {
    G3 = 0x00,
    G6 = 0x01,
    G12 = 0x02,
    G24 = 0x03,
}

#[derive(Debug, Clone, Copy)]
pub enum GyroRange {
    Dps2000 = 0x00,
    Dps1000 = 0x01,
    Dps500 = 0x02,
    Dps250 = 0x03,
    Dps125 = 0x04,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum AccelOdr {
    Hz12_5 = 0x05,
    Hz25 = 0x06,
    Hz50 = 0x07,
    Hz100 = 0x08,
    Hz200 = 0x09,
    Hz400 = 0x0A,
    Hz800 = 0x0B,
    Hz1600 = 0x0C,
}

#[derive(Debug, Clone, Copy, FromPrimitive, ToPrimitive)]
pub enum GyroOdr {
    Hz100 = 0x07,
    Hz200 = 0x06,
    Hz400 = 0x03,
    Hz1000 = 0x02,
    Hz2000 = 0x01,
}
