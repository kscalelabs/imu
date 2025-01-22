use log::error;
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Id, Socket};
use std::sync::{Arc, RwLock};
use std::thread;
use imu::data::{IMUData, Quaternion, EulerAngles, Vector3}; // Import the necessary structs

pub struct ImuReader {
    socket: Arc<CanSocket>,
    data: Arc<RwLock<IMUData>>,
    running: Arc<RwLock<bool>>,
}

impl ImuReader {
    pub fn new(
        interface: &str,
        serial_number: u8,
        model: u8,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let socket = Arc::new(CanSocket::open(interface)?);
        let data = Arc::new(RwLock::new(IMUData::default()));
        let running = Arc::new(RwLock::new(true));

        let imu_reader = ImuReader {
            socket: socket.clone(),
            data: Arc::clone(&data),
            running: Arc::clone(&running),
        };

        imu_reader.start_reading_thread(serial_number, model);

        Ok(imu_reader)
    }

    fn start_reading_thread(&self, serial_number: u8, model: u8) {
        let data: Arc<RwLock<IMUData>> = Arc::clone(&self.data);
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
                                let roll =
                                    i16::from_le_bytes([received_data[0], received_data[1]]) as f32
                                        * 0.01;
                                let pitch =
                                    i16::from_le_bytes([received_data[2], received_data[3]]) as f32
                                        * 0.01;
                                let yaw =
                                    i16::from_le_bytes([received_data[4], received_data[5]]) as f32
                                        * 0.01;

                                if let Ok(mut imu_data) = data.write() {
                                    imu_data.euler = EulerAngles { roll, pitch, yaw };
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
                                    imu_data.gyroscope = Vector3 {
                                        x: x_velocity,
                                        y: y_velocity,
                                        z: z_velocity,
                                    };
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
                                    imu_data.accelerometer = Vector3 {
                                        x: accel_x,
                                        y: accel_y,
                                        z: accel_z,
                                    };
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
                                    imu_data.quaternion.w = qw;
                                    imu_data.quaternion.x = qx;
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
                                    imu_data.quaternion.y = qy;
                                    imu_data.quaternion.z = qz;
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

    pub fn get_data(&self) -> Result<IMUData, String> {
        let data = self.data.read().map_err(|e| format!("Failed to read data: {}", e))?;
        Ok(*data)
    }

    pub fn get_angles(&self) -> Result<(f32, f32, f32), String> {
        let data = self.get_data()?;
        Ok((
            data.euler.roll - data.euler_offset.roll,
            data.euler.pitch - data.euler_offset.pitch,
            data.euler.yaw - data.euler_offset.yaw,
        ))
    }

    pub fn get_velocities(&self) -> Result<(f32, f32, f32), String> {
        let data = self.get_data()?;
        Ok((data.gyroscope.x, data.gyroscope.y, data.gyroscope.z))
    }

    pub fn get_accelerations(&self) -> Result<(f32, f32, f32), String> {
        let data = self.get_data()?;
        Ok((data.accelerometer.x, data.accelerometer.y, data.accelerometer.z))
    }

    pub fn get_quaternion(&self) -> Result<(f32, f32, f32, f32), String> {
        let data = self.get_data()?;
        Ok((data.quaternion.w, data.quaternion.x, data.quaternion.y, data.quaternion.z))
    }

    pub fn stop(&self) -> Result<(), String> {
        let mut running = self
            .running
            .write()
            .map_err(|e| format!("Failed to acquire write lock: {}", e))?;
        *running = false;
        Ok(())
    }

    pub fn zero_imu(
        &self,
        duration_ms: Option<u64>,
        max_retries: Option<u32>,
        max_variance: Option<f32>,
    ) -> Result<(), String> {
        let duration = duration_ms.unwrap_or(1000);
        let retries = max_retries.unwrap_or(3);
        let samples = duration / 10; // Sample every 10ms
        let max_variance = max_variance.unwrap_or(2.0); // Maximum allowed variance in degrees during calibration

        for attempt in 0..retries {
            let mut x_samples = Vec::new();
            let mut y_samples = Vec::new();
            let mut z_samples = Vec::new();

            // Collect samples
            for _ in 0..samples {
                let data = self.get_data()?;
                x_samples.push(data.euler.roll);
                y_samples.push(data.euler.pitch);
                z_samples.push(data.euler.yaw);
                thread::sleep(std::time::Duration::from_millis(10));
            }

            // Calculate variance
            let (x_var, y_var, z_var) = calculate_variance(&x_samples, &y_samples, &z_samples);

            if x_var > max_variance || y_var > max_variance || z_var > max_variance {
                if attempt == retries - 1 {
                    return Err(format!(
                        "Calibration failed after {} attempts. IMU must remain still during calibration.",
                        retries
                    ));
                }
                println!("Calibration failed because of high angular variance. Retrying...");
                continue;
            }

            // Calculate average offsets
            let x_offset = x_samples.iter().sum::<f32>() / samples as f32;
            let y_offset = y_samples.iter().sum::<f32>() / samples as f32;
            let z_offset = z_samples.iter().sum::<f32>() / samples as f32;

            // Update offsets

            let mut imu_data = self
                .data
                .write()
                .map_err(|e| format!("Failed to acquire write lock: {}", e))?;

            imu_data.euler_offset.roll = x_offset;
            imu_data.euler_offset.pitch = y_offset;
            imu_data.euler_offset.yaw = z_offset;

            return Ok(());
        }

        Err("Unexpected calibration failure".to_string())
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

impl Drop for ImuReader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
