#[derive(Debug, Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Vector3 {
    fn default() -> Self {
        Vector3 { x: 0.0, y: 0.0, z: 0.0 }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct EulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl Default for EulerAngles {
    fn default() -> Self {
        EulerAngles { roll: 0.0, pitch: 0.0, yaw: 0.0 }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Quaternion { w: 0.0, x: 0.0, y: 0.0, z: 0.0 }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct IMUData {
    pub quaternion: Quaternion,
    pub euler: EulerAngles,
    pub euler_offset: EulerAngles, // Used for hexmove IMU
    pub accelerometer: Vector3,
    pub gyroscope: Vector3,
    pub magnetometer: Vector3,
    pub linear_acceleration: Vector3,
    pub gravity: Vector3,
    pub temperature: i8,
    pub calibration_status: u8,
}

impl Default for IMUData {
    fn default() -> Self {
        IMUData {
            quaternion: Quaternion::default(),
            euler: EulerAngles::default(),
            euler_offset: EulerAngles::default(),
            accelerometer: Vector3::default(),
            gyroscope: Vector3::default(),
            magnetometer: Vector3::default(),
            linear_acceleration: Vector3::default(),
            gravity: Vector3::default(),
            temperature: 0,
            calibration_status: 0,
        }
    }
} 