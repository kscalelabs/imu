// Read and write frame types

use imu_traits::ImuError;
use std::convert::TryInto;

const START_BYTE: u8 = 0x55;
const PACKET_SIZE: usize = 11; // data packets at 11 bytes

pub struct RawFrame {
    pub frame_type: FrameType,
    pub data: [u8; 8],
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    Time = 0x50,
    Acceleration = 0x51,
    Gyro = 0x52,
    Angle = 0x53,
    Magnetometer = 0x54,
    PortStatus = 0x55,
    BaroAltitude = 0x56,
    LatLon = 0x57,
    Gps = 0x58,
    Quaternion = 0x59,
    GpsAccuracy = 0x5A,
    GenericRead = 0x5F,
}

impl FrameType {
    pub fn get_constant_value(self) -> f32 {
        match self {
            FrameType::Gyro => 2000.0 * std::f32::consts::PI / 180.0,
            FrameType::Angle => std::f32::consts::PI,
            FrameType::Acceleration => 16.0 * 9.80665,
            _ => 1.0,
        }
    }
}

impl TryFrom<u8> for FrameType {
    type Error = ImuError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x50 => Ok(FrameType::Time),
            0x51 => Ok(FrameType::Acceleration),
            0x52 => Ok(FrameType::Gyro),
            0x53 => Ok(FrameType::Angle),
            0x54 => Ok(FrameType::Magnetometer),
            0x55 => Ok(FrameType::PortStatus),
            0x56 => Ok(FrameType::BaroAltitude),
            0x57 => Ok(FrameType::LatLon),
            0x58 => Ok(FrameType::Gps),
            0x59 => Ok(FrameType::Quaternion),
            0x5A => Ok(FrameType::GpsAccuracy),
            0x5F => Ok(FrameType::GenericRead),
            _ => Err(ImuError::ReadError(format!(
                "Unknown frame type: {}",
                value
            ))),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ReadFrame {
    Time {
        year: u8,
        month: u8,
        day: u8,
        hour: u8,
        minute: u8,
        second: u8,
        ms: u16,
    },
    Acceleration {
        x: f32,
        y: f32,
        z: f32,
        temp: f32,
    },
    Gyro {
        x: f32,
        y: f32,
        z: f32,
        voltage: f32,
    },
    Angle {
        roll: f32,
        pitch: f32,
        yaw: f32,
        version: f32,
    },
    Magnetometer {
        x: f32,
        y: f32,
        z: f32,
        temp: f32,
    },
    PortStatus {
        d0: u16,
        d1: u16,
        d2: u16,
        d3: u16,
    },
    BaroAltitude {
        pressure: u32,
        height_cm: u32,
    },
    LatLon {
        lon: f64,
        lat: f64,
    },
    Gps {
        altitude: f32,
        heading: f32,
        ground_speed: f32,
    },
    Quaternion {
        w: f32,
        x: f32,
        y: f32,
        z: f32,
    },
    GpsAccuracy {
        sv: u16,
        pdop: f32,
        hdop: f32,
        vdop: f32,
    },
    GenericRead {
        data: [u8; 8],
    },
}

impl ReadFrame {
    pub fn get_type(self) -> FrameType {
        match self {
            ReadFrame::Time { .. } => FrameType::Time,
            ReadFrame::Acceleration { .. } => FrameType::Acceleration,
            ReadFrame::Gyro { .. } => FrameType::Gyro,
            ReadFrame::Angle { .. } => FrameType::Angle,
            ReadFrame::Magnetometer { .. } => FrameType::Magnetometer,
            ReadFrame::PortStatus { .. } => FrameType::PortStatus,
            ReadFrame::BaroAltitude { .. } => FrameType::BaroAltitude,
            ReadFrame::LatLon { .. } => FrameType::LatLon,
            ReadFrame::Gps { .. } => FrameType::Gps,
            ReadFrame::Quaternion { .. } => FrameType::Quaternion,
            ReadFrame::GpsAccuracy { .. } => FrameType::GpsAccuracy,
            ReadFrame::GenericRead { .. } => FrameType::GenericRead,
        }
    }

    pub fn deserialize(frame: RawFrame) -> Result<Self, ImuError> {
        let frame_type = frame.frame_type;
        let k = frame_type.get_constant_value();
        match frame_type {
            FrameType::Time => {
                let year = frame.data[0];
                let month = frame.data[1];
                let day = frame.data[2];
                let hour = frame.data[3];
                let minute = frame.data[4];
                let second = frame.data[5];
                let ms = u16::from(frame.data[7]) << 8 | u16::from(frame.data[6]);
                Ok(ReadFrame::Time {
                    year,
                    month,
                    day,
                    hour,
                    minute,
                    second,
                    ms,
                })
            }
            FrameType::Acceleration => {
                let acc_x = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let acc_y = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let acc_z = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let temp = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Acceleration {
                    x: acc_x as f32 / 32768.0 * k,
                    y: acc_y as f32 / 32768.0 * k,
                    z: acc_z as f32 / 32768.0 * k,
                    temp: temp as f32 / 100.0,
                })
            }
            // Returns radians per second
            FrameType::Gyro => {
                let gyro_x = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let gyro_y = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let gyro_z = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let voltage = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Gyro {
                    x: gyro_x as f32 / 32768.0 * k,
                    y: gyro_y as f32 / 32768.0 * k,
                    z: gyro_z as f32 / 32768.0 * k,
                    voltage: voltage as f32 / 100.0, // "Non-Bluetooth Products，the data is invalid" - https://github.com/YahboomTechnology/10-axis_IMU_Module
                })
            }
            FrameType::Angle => {
                let angle_x = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let angle_y = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let angle_z = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let version = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Angle {
                    roll: angle_x as f32 / 32768.0 * k,
                    pitch: angle_y as f32 / 32768.0 * k,
                    yaw: angle_z as f32 / 32768.0 * k,
                    version: version as f32,
                })
            }
            FrameType::Magnetometer => {
                let mag_x = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let mag_y = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let mag_z = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let temp = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Magnetometer {
                    x: mag_x as f32 / 32768.0 * k,
                    y: mag_y as f32 / 32768.0 * k,
                    z: mag_z as f32 / 32768.0 * k,
                    temp: temp as f32 / 100.0,
                })
            }
            FrameType::PortStatus => {
                let d0 = u16::from(frame.data[1]) << 8 | u16::from(frame.data[0]);
                let d1 = u16::from(frame.data[3]) << 8 | u16::from(frame.data[2]);
                let d2 = u16::from(frame.data[5]) << 8 | u16::from(frame.data[4]);
                let d3 = u16::from(frame.data[7]) << 8 | u16::from(frame.data[6]);
                Ok(ReadFrame::PortStatus { d0, d1, d2, d3 })
            }
            FrameType::BaroAltitude => {
                let pressure = u32::from(frame.data[3]) << 24
                    | u32::from(frame.data[2]) << 16
                    | u32::from(frame.data[1]) << 8
                    | u32::from(frame.data[0]);
                let height_cm = u32::from(frame.data[7]) << 24
                    | u32::from(frame.data[6]) << 16
                    | u32::from(frame.data[5]) << 8
                    | u32::from(frame.data[4]);
                Ok(ReadFrame::BaroAltitude {
                    pressure,
                    height_cm,
                })
            }
            FrameType::LatLon => {
                let longitude = i32::from(frame.data[3]) << 24
                    | i32::from(frame.data[2]) << 16
                    | i32::from(frame.data[1]) << 8
                    | i32::from(frame.data[0]);
                let latitude = i32::from(frame.data[7]) << 24
                    | i32::from(frame.data[6]) << 16
                    | i32::from(frame.data[5]) << 8
                    | i32::from(frame.data[4]);
                Ok(ReadFrame::LatLon {
                    lon: longitude as f64,
                    lat: latitude as f64,
                })
            }
            FrameType::Gps => {
                let altitude = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let heading = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let ground_speed = i32::from(frame.data[7]) << 24
                    | i32::from(frame.data[6]) << 16
                    | i32::from(frame.data[5]) << 8
                    | i32::from(frame.data[4]);
                Ok(ReadFrame::Gps {
                    altitude: altitude as f32 / 10.0,
                    heading: heading as f32 / 100.0,
                    ground_speed: ground_speed as f32 / 1000.0,
                })
            }
            FrameType::Quaternion => {
                let quat_w = i16::from(frame.data[1]) << 8 | i16::from(frame.data[0]);
                let quat_x = i16::from(frame.data[3]) << 8 | i16::from(frame.data[2]);
                let quat_y = i16::from(frame.data[5]) << 8 | i16::from(frame.data[4]);
                let quat_z = i16::from(frame.data[7]) << 8 | i16::from(frame.data[6]);
                Ok(ReadFrame::Quaternion {
                    w: quat_w as f32 / 32768.0 * k,
                    x: quat_x as f32 / 32768.0 * k,
                    y: quat_y as f32 / 32768.0 * k,
                    z: quat_z as f32 / 32768.0 * k,
                })
            }
            FrameType::GpsAccuracy => {
                let num_satellites = u16::from(frame.data[1]) << 8 | u16::from(frame.data[0]);
                let pdop = u16::from(frame.data[3]) << 8 | u16::from(frame.data[2]);
                let hdop = u16::from(frame.data[5]) << 8 | u16::from(frame.data[4]);
                let vdop = u16::from(frame.data[7]) << 8 | u16::from(frame.data[6]);
                Ok(ReadFrame::GpsAccuracy {
                    sv: num_satellites,
                    pdop: pdop as f32 / 100.0,
                    hdop: hdop as f32 / 100.0,
                    vdop: vdop as f32 / 100.0,
                })
            }
            FrameType::GenericRead => {
                let data = [
                    frame.data[0],
                    frame.data[1],
                    frame.data[2],
                    frame.data[3],
                    frame.data[4],
                    frame.data[5],
                    frame.data[6],
                    frame.data[7],
                ];
                Ok(ReadFrame::GenericRead { data })
            }
        }
    }
}

pub struct FrameParser {
    buffer: Vec<u8>,
}

impl FrameParser {
    pub fn new(buffer_size: Option<usize>) -> Self {
        let buffer_size = buffer_size.unwrap_or(512);
        FrameParser {
            buffer: Vec::with_capacity(buffer_size),
        }
    }

    /// Adds new bytes to the internal buffer and attempts to parse frames.
    /// Returns a Vec of successfully parsed frames.
    pub fn parse(&mut self, new_bytes: &[u8]) -> Result<Vec<ReadFrame>, ImuError> {
        self.buffer.extend_from_slice(new_bytes);

        let mut frames = Vec::new();
        let mut consumed = 0;

        // Process buffer seeking valid packets
        'outer: loop {
            // Find the next potential start byte (0x55) from the current search position
            if let Some(start_offset) = self.buffer[consumed..]
                .iter()
                .position(|&b| b == START_BYTE)
            {
                let packet_start_index = consumed + start_offset;

                if self.buffer.len() < packet_start_index + PACKET_SIZE {
                    consumed = packet_start_index;
                    break 'outer;
                }

                let packet_bytes =
                    &self.buffer[packet_start_index..packet_start_index + PACKET_SIZE];

                // Calculate checksum (Start + Type + Data[0..8])
                let calculated_checksum: u8 = packet_bytes[0..10]
                    .iter()
                    .fold(0, |acc, &x| acc.wrapping_add(x));
                let received_checksum = packet_bytes[10];

                if calculated_checksum == received_checksum {
                    let type_byte = packet_bytes[1];
                    let data_bytes: [u8; 8] = packet_bytes[2..10].try_into().map_err(|_| {
                        ImuError::ReadError("Failed to convert slice to array".to_string())
                    })?;

                    match FrameType::try_from(type_byte) {
                        Ok(frame_type) => {
                            let raw_frame = RawFrame {
                                frame_type,
                                data: data_bytes,
                            };
                            match ReadFrame::deserialize(raw_frame) {
                                // Assumes deserialize is implemented
                                Ok(parsed_frame) => {
                                    frames.push(parsed_frame);
                                    consumed = packet_start_index + PACKET_SIZE;
                                }
                                Err(e) => {
                                    eprintln!(
                                        "Frame deserialization error: {:?}, discarding packet.",
                                        e
                                    );
                                    consumed = packet_start_index + PACKET_SIZE;
                                }
                            }
                        }
                        Err(e) => {
                            eprintln!("Unknown frame type byte error: {:?}, discarding packet.", e);
                            consumed = packet_start_index + PACKET_SIZE;
                        }
                    }
                } else {
                    eprintln!("Checksum mismatch, discarding packet.");
                    consumed = packet_start_index + 1;
                }
            } else {
                // No more start bytes found, consume all remaining bytes
                consumed = self.buffer.len();
                break 'outer;
            }
        }

        // Drain the consumed part of the buffer
        if consumed > 0 {
            self.buffer.drain(0..consumed);
        }

        Ok(frames)
    }

    pub fn clear_buffer(&mut self) {
        self.buffer.clear();
    }
}
