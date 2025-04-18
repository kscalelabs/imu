pub use imu_traits::{ImuData, ImuError, ImuReader, Quaternion, Vector3};
use log::error;
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Id, Socket};
use std::sync::{Arc, RwLock};
use std::thread;

pub struct HexmoveImuReader {
    socket: Arc<CanSocket>,
    data: Arc<RwLock<ImuData>>,
    running: Arc<RwLock<bool>>,
}

impl HexmoveImuReader {
    pub fn new(interface: &str, serial_number: u8, model: u8) -> Result<Self, ImuError> {
        let socket = Arc::new(CanSocket::open(interface)?);
        let data = Arc::new(RwLock::new(ImuData::default()));
        let running = Arc::new(RwLock::new(true));

        let imu_reader = HexmoveImuReader {
            socket: socket.clone(),
            data: Arc::clone(&data),
            running: Arc::clone(&running),
        };

        imu_reader.start_reading_thread(serial_number, model);

        Ok(imu_reader)
    }

    fn start_reading_thread(&self, serial_number: u8, model: u8) {
        let data = Arc::clone(&self.data);
        let running = Arc::clone(&self.running);
        let socket = Arc::clone(&self.socket);

        thread::spawn(move || {
            loop {
                // Check if we should continue running
                if let Ok(guard) = running.read() {
                    if !*guard {
                        break;
                    }
                } else {
                    error!("Failed to acquire read lock");
                    break;
                }

                match socket.read_frame() {
                    Ok(CanFrame::Data(data_frame)) => {
                        let received_data = data_frame.data();
                        let id = data_frame.id();

                        let base_id =
                            0x0B000000 | (serial_number as u32) << 16 | (model as u32) << 8;

                        // IMU angle data
                        if let Some(ext_id) = ExtendedId::new(base_id | 0xB1) {
                            if id == Id::Extended(ext_id) {
                                let x_angle =
                                    i16::from_le_bytes([received_data[0], received_data[1]]) as f32
                                        * 0.01;
                                let y_angle =
                                    i16::from_le_bytes([received_data[2], received_data[3]]) as f32
                                        * 0.01;
                                let z_angle =
                                    i16::from_le_bytes([received_data[4], received_data[5]]) as f32
                                        * 0.01;

                                if let Ok(mut imu_data) = data.write() {
                                    imu_data.euler = Some(Vector3 {
                                        x: x_angle,
                                        y: y_angle,
                                        z: z_angle,
                                    });
                                } else {
                                    error!("Failed to write to IMU data");
                                }
                            }
                        } else {
                            error!("Failed to create extended ID for IMU data");
                        }

                        // IMU velocity data
                        if let Some(ext_id) = ExtendedId::new(base_id | 0xB2) {
                            if id == Id::Extended(ext_id) {
                                let x_velocity =
                                    i16::from_le_bytes([received_data[0], received_data[1]]) as f32
                                        * 0.01;
                                let y_velocity =
                                    i16::from_le_bytes([received_data[2], received_data[3]]) as f32
                                        * 0.01;
                                let z_velocity =
                                    i16::from_le_bytes([received_data[4], received_data[5]]) as f32
                                        * 0.01;

                                if let Ok(mut imu_data) = data.write() {
                                    imu_data.gyroscope = Some(Vector3 {
                                        x: x_velocity,
                                        y: y_velocity,
                                        z: z_velocity,
                                    });
                                } else {
                                    error!("Failed to write to IMU data");
                                }
                            }
                        } else {
                            error!("Failed to create extended ID for IMU velocity data");
                        }

                        // IMU acceleration data (m/s^2)
                        if let Some(ext_id) = ExtendedId::new(base_id | 0xB3) {
                            if id == Id::Extended(ext_id) {
                                let accel_x =
                                    i16::from_le_bytes([received_data[0], received_data[1]]) as f32
                                        * 0.01;
                                let accel_y =
                                    i16::from_le_bytes([received_data[2], received_data[3]]) as f32
                                        * 0.01;
                                let accel_z =
                                    i16::from_le_bytes([received_data[4], received_data[5]]) as f32
                                        * 0.01;

                                if let Ok(mut imu_data) = data.write() {
                                    imu_data.accelerometer = Some(Vector3 {
                                        x: accel_x,
                                        y: accel_y,
                                        z: accel_z,
                                    });
                                } else {
                                    error!("Failed to write to IMU data");
                                }
                            }
                        } else {
                            error!("Failed to create extended ID for IMU acceleration data");
                        }

                        // IMU quaternion data
                        if let Some(ext_id) = ExtendedId::new(base_id | 0xB4) {
                            if id == Id::Extended(ext_id) {
                                // Parse quaternion w component from first 4 bytes
                                let qw_bytes: [u8; 4] = [
                                    received_data[0],
                                    received_data[1],
                                    received_data[2],
                                    received_data[3],
                                ];
                                let qw = f32::from_le_bytes(qw_bytes);

                                // Parse quaternion x component from last 4 bytes
                                let qx_bytes: [u8; 4] = [
                                    received_data[4],
                                    received_data[5],
                                    received_data[6],
                                    received_data[7],
                                ];
                                let qx = f32::from_le_bytes(qx_bytes);

                                if let Ok(mut imu_data) = data.write() {
                                    let current =
                                        imu_data.quaternion.unwrap_or(Quaternion::default());
                                    imu_data.quaternion = Some(Quaternion {
                                        w: qw,
                                        x: qx,
                                        y: current.y,
                                        z: current.z,
                                    });
                                } else {
                                    error!("Failed to write quaternion data to IMU data");
                                }
                            }
                        } else {
                            error!("Failed to create extended ID for IMU quaternion data");
                        }

                        // IMU quaternion data
                        if let Some(ext_id) = ExtendedId::new(base_id | 0xB5) {
                            if id == Id::Extended(ext_id) {
                                // Parse quaternion y component from first 4 bytes
                                let qy_bytes: [u8; 4] = [
                                    received_data[0],
                                    received_data[1],
                                    received_data[2],
                                    received_data[3],
                                ];
                                let qy = f32::from_le_bytes(qy_bytes);

                                // Parse quaternion z component from last 4 bytes
                                let qz_bytes: [u8; 4] = [
                                    received_data[4],
                                    received_data[5],
                                    received_data[6],
                                    received_data[7],
                                ];
                                let qz = f32::from_le_bytes(qz_bytes);

                                if let Ok(mut imu_data) = data.write() {
                                    let current =
                                        imu_data.quaternion.unwrap_or(Quaternion::default());
                                    imu_data.quaternion = Some(Quaternion {
                                        w: current.w,
                                        x: current.x,
                                        y: qy,
                                        z: qz,
                                    });
                                } else {
                                    error!("Failed to write quaternion data to IMU data");
                                }
                            }
                        } else {
                            error!("Failed to create extended ID for IMU quaternion data");
                        }
                    }
                    Ok(CanFrame::Remote(_)) => {
                        // Ignore remote frames
                    }
                    Ok(CanFrame::Error(_)) => {
                        // Ignore error frames
                    }
                    Err(e) => {
                        error!("Error reading IMU data: {}", e);
                    }
                }
            }
        });
    }

    pub fn get_angles(&self) -> Result<(f32, f32, f32), ImuError> {
        let data = self.get_data()?;
        let euler = data
            .euler
            .ok_or(ImuError::ReadError("No euler data".to_string()))?;
        Ok((euler.x, euler.y, euler.z))
    }

    pub fn get_velocities(&self) -> Result<(f32, f32, f32), ImuError> {
        let data = self.get_data()?;
        let gyro = data
            .gyroscope
            .ok_or(ImuError::ReadError("No gyroscope data".to_string()))?;
        Ok((gyro.x, gyro.y, gyro.z))
    }

    pub fn get_accelerations(&self) -> Result<(f32, f32, f32), ImuError> {
        let data = self.get_data()?;
        let accel = data
            .accelerometer
            .ok_or(ImuError::ReadError("No accelerometer data".to_string()))?;
        Ok((accel.x, accel.y, accel.z))
    }

    pub fn get_quaternion(&self) -> Result<(f32, f32, f32, f32), ImuError> {
        let data = self.get_data()?;
        let quaternion = data
            .quaternion
            .ok_or(ImuError::ReadError("No quaternion data".to_string()))?;
        Ok((quaternion.w, quaternion.x, quaternion.y, quaternion.z))
    }
}

impl ImuReader for HexmoveImuReader {
    fn get_data(&self) -> Result<ImuData, ImuError> {
        let imu_data = self
            .data
            .read()
            .map_err(|e| ImuError::LockError(e.to_string()))?;
        Ok(imu_data.clone())
    }

    fn stop(&self) -> Result<(), ImuError> {
        let mut running = self
            .running
            .write()
            .map_err(|e| ImuError::LockError(e.to_string()))?;
        *running = false;
        Ok(())
    }
}

fn calculate_variance(x: &[f32], y: &[f32], z: &[f32]) -> (f32, f32, f32) {
    let x_mean = x.iter().sum::<f32>() / x.len() as f32;
    let y_mean = y.iter().sum::<f32>() / y.len() as f32;
    let z_mean = z.iter().sum::<f32>() / z.len() as f32;

    let x_var = x.iter().map(|v| (v - x_mean).powi(2)).sum::<f32>() / x.len() as f32;
    let y_var = y.iter().map(|v| (v - y_mean).powi(2)).sum::<f32>() / y.len() as f32;
    let z_var = z.iter().map(|v| (v - z_mean).powi(2)).sum::<f32>() / z.len() as f32;

    (x_var, y_var, z_var)
}

impl Drop for HexmoveImuReader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
