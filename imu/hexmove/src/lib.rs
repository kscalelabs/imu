use log::error;
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Id, Socket};
use std::sync::{Arc, RwLock};
use std::thread;

#[derive(Debug, Default, Clone)]
pub struct ImuData {
    pub x_angle: f32,
    pub y_angle: f32,
    pub z_angle: f32,
    pub x_velocity: f32,
    pub y_velocity: f32,
    pub z_velocity: f32,
    pub x_angle_offset: f32,
    pub y_angle_offset: f32,
    pub z_angle_offset: f32,
}

pub struct ImuReader {
    socket: Arc<CanSocket>,
    data: Arc<RwLock<ImuData>>,
    running: Arc<RwLock<bool>>,
}

impl ImuReader {
    pub fn new(
        interface: &str,
        serial_number: u8,
        model: u8,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let socket = Arc::new(CanSocket::open(interface)?);
        let data = Arc::new(RwLock::new(ImuData::default()));
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
        let data = Arc::clone(&self.data);
        let running = Arc::clone(&self.running);
        let socket = Arc::clone(&self.socket);

        thread::spawn(move || {
            while *running.read().unwrap() {
                match socket.read_frame() {
                    Ok(CanFrame::Data(data_frame)) => {
                        let received_data = data_frame.data();
                        let id = data_frame.id();

                        let base_id =
                            0x0B000000 | (serial_number as u32) << 16 | (model as u32) << 8;

                        if id == Id::Extended(ExtendedId::new(base_id | 0xB1).unwrap()) {
                            let x_angle = i16::from_le_bytes([received_data[0], received_data[1]])
                                as f32
                                * 0.01;
                            let y_angle = i16::from_le_bytes([received_data[2], received_data[3]])
                                as f32
                                * 0.01;
                            let z_angle = i16::from_le_bytes([received_data[4], received_data[5]])
                                as f32
                                * 0.01;

                            let mut imu_data = data.write().unwrap();
                            imu_data.x_angle = x_angle;
                            imu_data.y_angle = y_angle;
                            imu_data.z_angle = z_angle;
                        }

                        if id == Id::Extended(ExtendedId::new(base_id | 0xB2).unwrap()) {
                            let x_velocity =
                                i16::from_le_bytes([received_data[0], received_data[1]]) as f32
                                    * 0.01;
                            let y_velocity =
                                i16::from_le_bytes([received_data[2], received_data[3]]) as f32
                                    * 0.01;
                            let z_velocity =
                                i16::from_le_bytes([received_data[4], received_data[5]]) as f32
                                    * 0.01;

                            let mut imu_data = data.write().unwrap();
                            imu_data.x_velocity = x_velocity;
                            imu_data.y_velocity = y_velocity;
                            imu_data.z_velocity = z_velocity;
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

    pub fn get_data(&self) -> ImuData {
        self.data.read().unwrap().clone()
    }

    pub fn get_angles(&self) -> (f32, f32, f32) {
        let data = self.get_data();
        (
            data.x_angle - data.x_angle_offset,
            data.y_angle - data.y_angle_offset,
            data.z_angle - data.z_angle_offset,
        )
    }

    pub fn get_velocities(&self) -> (f32, f32, f32) {
        let data = self.get_data();
        (data.x_velocity, data.y_velocity, data.z_velocity)
    }

    pub fn stop(&self) {
        let mut running = self.running.write().unwrap();
        *running = false;
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
                let data = self.get_data();
                x_samples.push(data.x_angle);
                y_samples.push(data.y_angle);
                z_samples.push(data.z_angle);
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
            let mut imu_data = self.data.write().unwrap();
            imu_data.x_angle_offset = x_offset;
            imu_data.y_angle_offset = y_offset;
            imu_data.z_angle_offset = z_offset;

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
        self.stop();
    }
}
