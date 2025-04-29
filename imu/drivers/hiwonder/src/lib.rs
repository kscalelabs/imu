pub mod frame;
pub mod register;
pub use frame::*;
pub use imu_traits::{ImuData, ImuError, ImuFrequency, ImuReader, Quaternion, Vector3};
pub use register::*;
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::{Duration, Instant};
use strum::IntoEnumIterator;
use tracing::{debug, error, info, trace, warn};

pub trait FrequencyToByte {
    fn to_byte(&self) -> u8;
}

impl FrequencyToByte for ImuFrequency {
    fn to_byte(&self) -> u8 {
        match self {
            ImuFrequency::Hz0_2 => 0x01,
            ImuFrequency::Hz0_5 => 0x02,
            ImuFrequency::Hz1 => 0x03,
            ImuFrequency::Hz2 => 0x04,
            ImuFrequency::Hz5 => 0x05,
            ImuFrequency::Hz10 => 0x06,
            ImuFrequency::Hz20 => 0x07,
            ImuFrequency::Hz50 => 0x08,
            ImuFrequency::Hz100 => 0x09,
            ImuFrequency::Hz200 => 0x0B,
            ImuFrequency::Single => 0x0C,
            ImuFrequency::None => 0x0D,
        }
    }
}

#[derive(PartialEq, Eq, Debug)]
pub enum ImuMode {
    Read,
    Write,
}

pub struct HiwonderReader {
    data: Arc<RwLock<ImuData>>,
    mode: Arc<RwLock<ImuMode>>,
    port: Arc<Mutex<Box<dyn serialport::SerialPort>>>,
    running: Arc<RwLock<bool>>,
    frame_parser: Arc<Mutex<FrameParser>>,
}

#[derive(Debug)]
pub enum ImuCommand {
    Reset,
    Stop,
    SetFrequency(ImuFrequency),
    SetBaudRate(u32),
}

impl HiwonderReader {
    pub fn new(
        interface: &str,
        desired_baud_rate: u32,
        timeout: Duration,
        auto_detect_baud_rate: bool,
    ) -> Result<Self, ImuError> {
        let data = Arc::new(RwLock::new(ImuData::default()));
        let running = Arc::new(RwLock::new(true));
        let frame_parser_arc = Arc::new(Mutex::new(FrameParser::new(Some(512))));

        let final_port: Box<dyn serialport::SerialPort>;
        let _final_baud_rate: u32;

        if auto_detect_baud_rate {
            (final_port, _final_baud_rate) = Self::detect_and_set_baud_rate(
                interface,
                desired_baud_rate,
                Duration::from_secs(10),
            )
            .map_err(|e| {
                ImuError::ConfigurationError(format!("Baud rate detection/setting failed: {}", e))
            })?;
        } else {
            final_port = serialport::new(interface, desired_baud_rate)
                .timeout(timeout)
                .open()
                .map_err(|e| {
                    ImuError::ConfigurationError(format!(
                        "Failed to open port directly at {}: {}",
                        desired_baud_rate, e
                    ))
                })?;
            _final_baud_rate = desired_baud_rate;
        }

        let port_arc = Arc::new(Mutex::new(final_port));

        let reader = HiwonderReader {
            data: Arc::clone(&data),
            mode: Arc::new(RwLock::new(ImuMode::Read)),
            port: Arc::clone(&port_arc),
            running: Arc::clone(&running),
            frame_parser: frame_parser_arc,
        };

        if let Ok(mut guard) = reader.frame_parser.lock() {
            guard.clear_buffer();
        }

        reader.start_reading_thread()?;

        Ok(reader)
    }

    /// Detects the current baud rate, sets it to the desired rate if necessary,
    /// and returns the opened serial port configured at the desired baud rate.
    fn detect_and_set_baud_rate(
        interface: &str,
        desired_baud_rate: u32,
        timeout: Duration,
    ) -> Result<(Box<dyn serialport::SerialPort>, u32), ImuError> {
        let mut baud_rates_to_try = Vec::new();
        baud_rates_to_try.push(BaudRate::try_from(desired_baud_rate).map_err(|e| {
            ImuError::ConfigurationError(format!(
                "Desired baud rate {} invalid: {}",
                desired_baud_rate, e
            ))
        })?);

        let all_baud_rates = [
            BaudRate::Baud4800,
            BaudRate::Baud9600,
            BaudRate::Baud19200,
            BaudRate::Baud38400,
            BaudRate::Baud57600,
            BaudRate::Baud115200,
            BaudRate::Baud230400,
            BaudRate::Baud460800,
            BaudRate::Baud921600,
        ];

        for baud_rate in all_baud_rates {
            baud_rates_to_try.push(baud_rate);
        }

        let mut detected_port: Option<Box<dyn serialport::SerialPort>> = None;
        let mut current_baud_rate: Option<u32> = None;

        let overall_start_time = Instant::now();

        let time_per_baud = timeout
            .checked_div(baud_rates_to_try.len() as u32 + 10)
            .unwrap_or(Duration::from_millis(200));

        info!("Attempting to detect IMU baud rate...");

        for test_baud in baud_rates_to_try.iter() {
            if overall_start_time.elapsed() > timeout {
                return Err(ImuError::ConfigurationError(format!(
                    "Baud rate detection timed out after {:?}",
                    timeout
                )));
            }
            debug!("Trying baud rate: {:?}", test_baud);

            let baud_rate = test_baud.clone().try_into().map_err(|e| {
                ImuError::ConfigurationError(format!("Failed to convert baud rate: {}", e))
            })?;
            match serialport::new(interface, baud_rate)
                .timeout(time_per_baud)
                .open()
            {
                Ok(mut port) => {
                    debug!("Port opened at {}", baud_rate);
                    let mut temp_parser = FrameParser::new(Some(512));

                    let unlock_cmd = UnlockCommand::new().to_bytes();
                    let read_cmd = ReadAddressCommand::new(Register::GpsBaud).to_bytes();

                    if port.write_all(&unlock_cmd).is_err() {
                        warn!("Failed to write unlock cmd at {}", baud_rate);
                        continue;
                    }
                    thread::sleep(Duration::from_millis(10));
                    if port.write_all(&read_cmd).is_err() {
                        warn!("Failed to write read cmd at {}", baud_rate);
                        continue;
                    }
                    debug!("Sent Unlock and Read command at {}", baud_rate);

                    // Attempt to read response
                    let mut buffer = [0u8; 256];
                    let read_start_time = Instant::now();
                    let mut found_response = false;
                    while read_start_time.elapsed() < time_per_baud {
                        match port.read(&mut buffer) {
                            Ok(n) if n > 0 => {
                                trace!("Read {} bytes at {}: {:?}", n, baud_rate, &buffer[..n]);
                                match temp_parser.parse(&buffer[..n]) {
                                    Ok(frames) => {
                                        if frames
                                            .iter()
                                            .any(|f| matches!(f, ReadFrame::GenericRead { .. }))
                                        {
                                            info!("Detected working baud rate: {}", baud_rate);
                                            found_response = true;
                                            break;
                                        }
                                    }
                                    Err(e) => trace!("Parse error at {}: {}", baud_rate, e),
                                }
                            }
                            Ok(_) => {} // No data
                            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                                trace!("Read timed out at {}, continuing check.", baud_rate);
                            }
                            Err(e) => {
                                warn!("Read error at {}: {}", baud_rate, e);
                                break;
                            }
                        }
                        thread::sleep(Duration::from_millis(10));
                    }

                    if found_response {
                        detected_port = Some(port);
                        current_baud_rate = Some(baud_rate);
                        break;
                    } else {
                        debug!("No valid response received at {}", baud_rate);
                    }
                }
                Err(e) => {
                    debug!("Failed to open port at {}: {}", baud_rate, e);
                }
            }
        }

        let mut final_port = match detected_port {
            Some(port) => port,
            None => {
                return Err(ImuError::ConfigurationError(
                    "Could not find working baud rate for IMU.".to_string(),
                ))
            }
        };

        let current_baud = match current_baud_rate {
            Some(baud) => baud,
            None => {
                return Err(ImuError::ConfigurationError(
                    "Failed to detect current IMU baud rate.".to_string(),
                ))
            }
        };

        if current_baud != desired_baud_rate {
            info!(
                "Current IMU baud rate is {}, changing to {}.",
                current_baud, desired_baud_rate
            );

            let desired_baud_enum = BaudRate::try_from(desired_baud_rate).map_err(|e| {
                ImuError::ConfigurationError(format!(
                    "Desired baud rate {} invalid: {}",
                    desired_baud_rate, e
                ))
            })?;

            let set_baud_cmd = SetBaudRateCommand::new(desired_baud_enum);
            let save_cmd = SaveCommand::new();
            let unlock_cmd = UnlockCommand::new();

            let command_timeout = Duration::from_secs(1);
            final_port.set_timeout(command_timeout).map_err(|e| {
                ImuError::ConfigurationError(format!("Failed to set write timeout: {}", e))
            })?;

            final_port.write_all(&unlock_cmd.to_bytes()).map_err(|e| {
                ImuError::WriteError(format!("Failed to write unlock before set baud: {}", e))
            })?;
            thread::sleep(Duration::from_millis(30));

            final_port
                .write_all(&set_baud_cmd.to_bytes())
                .map_err(|e| {
                    ImuError::WriteError(format!("Failed to write set baud rate command: {}", e))
                })?;

            final_port.write_all(&save_cmd.to_bytes()).map_err(|e| {
                ImuError::WriteError(format!(
                    "Failed to write save command after set baud: {}",
                    e
                ))
            })?;

            info!(
                "IMU baud rate change command sent. Re-opening port at {}.",
                desired_baud_rate
            );
            drop(final_port);
            thread::sleep(Duration::from_millis(200));

            final_port = serialport::new(interface, desired_baud_rate)
                .timeout(timeout)
                .open()
                .map_err(|e| {
                    ImuError::ConfigurationError(format!(
                        "Failed to re-open port at new baud rate {}: {}",
                        desired_baud_rate, e
                    ))
                })?;

            info!(
                "Port re-opened successfully at desired baud rate {}.",
                desired_baud_rate
            );
        } else {
            info!("IMU already at desired baud rate {}.", desired_baud_rate);
            final_port.set_timeout(timeout).map_err(|e| {
                ImuError::ConfigurationError(format!("Failed to set final timeout: {}", e))
            })?;
        }

        Ok((final_port, desired_baud_rate))
    }

    fn read_frames(
        port_arc: &Arc<Mutex<Box<dyn serialport::SerialPort>>>,
        parser_arc: &Arc<Mutex<FrameParser>>,
    ) -> Result<Vec<ReadFrame>, ImuError> {
        let mut buffer = [0u8; 1024];

        let mut port_guard = port_arc
            .lock()
            .map_err(|e| ImuError::ReadError(format!("Failed to acquire lock on port: {}", e)))?;

        let mut parser_guard = parser_arc.lock().map_err(|e| {
            ImuError::ReadError(format!("Failed to acquire lock on frame parser: {}", e))
        })?;

        match port_guard.read(&mut buffer) {
            Ok(n) => {
                if n > 0 {
                    parser_guard.parse(&buffer[0..n])
                } else {
                    warn!("No data read from port...");
                    Ok(vec![])
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(vec![]),
            Err(e) => Err(ImuError::ReadError(format!(
                "Failed to read data from port: {}",
                e
            ))),
        }
    }

    fn set_data(imu_data: &mut ImuData, frame: &ReadFrame) {
        match *frame {
            ReadFrame::Acceleration { x, y, z, temp: _ } => {
                imu_data.accelerometer = Some(Vector3 { x, y, z });
            }
            ReadFrame::Gyro {
                x,
                y,
                z,
                voltage: _,
            } => {
                imu_data.gyroscope = Some(Vector3 { x, y, z });
            }
            ReadFrame::Angle {
                roll,
                pitch,
                yaw,
                version: _,
            } => {
                imu_data.euler = Some(Vector3 {
                    x: roll,
                    y: pitch,
                    z: yaw,
                });
            }
            ReadFrame::Magnetometer { x, y, z, temp: _ } => {
                imu_data.magnetometer = Some(Vector3 { x, y, z });
            }
            ReadFrame::Quaternion { w, x, y, z } => {
                imu_data.quaternion = Some(Quaternion { w, x, y, z });
            }
            _ => (),
        }
    }

    fn start_reading_thread(&self) -> Result<(), ImuError> {
        let data = Arc::clone(&self.data);
        let running = Arc::clone(&self.running);
        let port = Arc::clone(&self.port);
        let frame_parser = Arc::clone(&self.frame_parser);
        let mode = Arc::clone(&self.mode);

        thread::spawn(move || {
            while let Ok(guard) = running.read() {
                if !*guard {
                    break;
                }

                if let Ok(mode) = mode.read() {
                    if *mode == ImuMode::Read {
                        match Self::read_frames(&port, &frame_parser) {
                            Ok(frames) => {
                                if frames.is_empty() {
                                    warn!("No frames read from port");
                                }
                                for frame in frames {
                                    if let Ok(mut imu_data) = data.write() {
                                        debug!("Setting data for frame: {:?}", frame);
                                        Self::set_data(&mut imu_data, &frame);
                                    } else {
                                        error!("Failed to write to IMU data");
                                    }
                                }
                            }
                            Err(e) => error!("Error reading/parsing frames in thread: {}", e),
                        }
                    } else {
                        debug!("IMU is in write mode");
                    }

                    thread::sleep(Duration::from_millis(4));
                }
            }
        });

        Ok(())
    }

    pub fn reset(&self) -> Result<(), ImuError> {
        if let Ok(mut running_guard) = self.running.write() {
            *running_guard = false;
            Ok(())
        } else {
            Err(ImuError::ReadError(
                "Failed to acquire lock for stop".to_string(),
            ))
        }
    }

    pub fn write_command(
        &self,
        command: &dyn BytableRegistrable,
        verify: bool,
        timeout: Duration,
    ) -> Result<(), ImuError> {
        {
            let mut mode_guard = self
                .mode
                .write()
                .map_err(|_| ImuError::WriteError("Mode lock poisoned".to_string()))?;
            *mode_guard = ImuMode::Write;
            debug!("Mode set to Write for register {:?}.", command.register());
        }

        struct ModeGuard<'a> {
            mode: &'a Arc<RwLock<ImuMode>>,
        }

        impl Drop for ModeGuard<'_> {
            fn drop(&mut self) {
                if let Ok(mut guard) = self.mode.write() {
                    *guard = ImuMode::Read; // Always set back to Read on exit
                    debug!("Mode set back to Read.");
                } else {
                    error!("Failed to acquire mode lock to set back to Read!");
                }
            }
        }

        let _mode_guard = ModeGuard { mode: &self.mode };

        let mut parser_guard = self.frame_parser.lock().map_err(|e| {
            ImuError::WriteError(format!("Failed to acquire lock on frame parser: {}", e))
        })?;

        // Clear parser buffer before writing/verifying
        parser_guard.clear_buffer();
        debug!("Cleared parser buffer.");

        let mut port_guard = self
            .port
            .lock()
            .map_err(|e| ImuError::WriteError(format!("Failed to acquire lock on port: {}", e)))?;

        let command_bytes = command.to_bytes();

        // Before sending the command, unlock the IMU
        port_guard
            .write_all(&UnlockCommand::new().to_bytes())
            .map_err(|e| ImuError::WriteError(format!("Failed to write unlock command: {}", e)))?;

        std::thread::sleep(Duration::from_millis(30));

        debug!("Sending command {:?} to IMU", command_bytes.as_slice());
        port_guard
            .write_all(&command_bytes)
            .map_err(|e| ImuError::WriteError(format!("Failed to write command: {}", e)))?;

        // Save the command
        port_guard
            .write_all(&SaveCommand::new().to_bytes())
            .map_err(|e| ImuError::WriteError(format!("Failed to write save command: {}", e)))?;

        debug!("Sent command {:?} to IMU", command_bytes.as_slice());

        if verify {
            std::thread::sleep(Duration::from_millis(100));
            let read_command_bytes = ReadAddressCommand::new(command.register()).to_bytes();
            port_guard.write_all(&read_command_bytes).map_err(|e| {
                ImuError::WriteError(format!(
                    "Failed to write read command for verification: {}",
                    e
                ))
            })?;
            std::thread::sleep(Duration::from_millis(5));

            info!(
                "Sent read command {:?} to IMU for verification of register {:?}",
                read_command_bytes.as_slice(),
                command.register()
            );

            let start_time = Instant::now();
            let mut buffer = [0u8; 1024]; // Buffer for verification read

            while start_time.elapsed() < timeout {
                match port_guard.read(&mut buffer) {
                    Ok(n) => {
                        if n > 0 {
                            trace!("Verification read {} bytes: {:?}", n, &buffer[..n]);
                            match parser_guard.parse(&buffer[0..n]) {
                                Ok(response) => {
                                    for frame in response {
                                        match frame {
                                            ReadFrame::GenericRead { data } => {
                                                debug!("Received generic read frame: {:?}", data);
                                                debug!("Command bytes: {:?}", command_bytes);
                                                // Expected data is the 2-byte value written, found at index 3 and 4 of the command bytes
                                                let expected_data = &command_bytes[3..5];
                                                // Check the first 2 bytes of the returned 8-byte data array
                                                if data[0..2] == expected_data[0..2] {
                                                    info!(
                                                        "Command for register {:?} verified. Wrote {:?}, Read back {:?}",
                                                        command.register(),
                                                        expected_data,
                                                        &data[0..2]
                                                    );
                                                    parser_guard.clear_buffer();
                                                    return Ok(());
                                                } else {
                                                    warn!(
                                                        "Command verification MISMATCH for register {:?}. Wrote {:?}, Read back {:?}. Full read data: {:?}",
                                                        command.register(),
                                                        expected_data,
                                                        &data[0..2],
                                                        data
                                                    );
                                                    parser_guard.clear_buffer();
                                                }
                                            }
                                            _ => trace!("Received unexpected frame during verification: {:?}", frame),
                                        }
                                    }
                                }
                                Err(e) => {
                                    error!("Failed to parse verification response: {}", e);
                                    parser_guard.clear_buffer();
                                }
                            }
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                        debug!("Verification read timed out, continuing wait...");
                    }
                    Err(e) => {
                        error!("Error reading verification response: {}", e);
                        parser_guard.clear_buffer();
                        return Err(ImuError::ReadError(format!(
                            "Verification read error: {}",
                            e
                        )));
                    }
                }
                std::thread::sleep(Duration::from_millis(20));
            }

            error!(
                "Command {:?} sent but verification timed out after {:?} for register {:?}",
                command_bytes.as_slice(),
                timeout,
                command.register()
            );
            parser_guard.clear_buffer(); // Clear buffer on timeout
            Err(ImuError::ReadError(format!(
                "Verification timed out for register {:?}",
                command.register()
            )))
        } else {
            debug!(
                "Command {:?} sent without verification.",
                command_bytes.as_slice()
            );
            parser_guard.clear_buffer();
            Ok(())
        }
    }

    pub fn set_frequency(
        &self,
        frequency: ImuFrequency,
        timeout: Duration,
    ) -> Result<(), ImuError> {
        self.write_command(&SetFrequencyCommand::new(frequency), true, timeout)?;
        Ok(())
    }

    pub fn set_baud_rate(&self, baud_rate: u32, timeout: Duration) -> Result<(), ImuError> {
        let baud_enum = BaudRate::try_from(baud_rate)?;
        self.write_command(&SetBaudRateCommand::new(baud_enum), true, timeout)?;
        Ok(())
    }

    pub fn set_bandwidth(&self, bandwidth: u32, timeout: Duration) -> Result<(), ImuError> {
        let bandwidth_enum = Bandwidth::try_from(bandwidth)?;
        self.write_command(&SetBandwidthCommand::new(bandwidth_enum), true, timeout)?;
        Ok(())
    }

    pub fn set_output_mode(&self, mode: Output, timeout: Duration) -> Result<(), ImuError> {
        self.write_command(&EnableOutputCommand::new(mode), true, timeout)?;
        Ok(())
    }

    pub fn read_register(
        &self,
        register: Register,
        timeout: Duration,
    ) -> Result<[u8; 8], ImuError> {
        let mut port_guard = self
            .port
            .lock()
            .map_err(|e| ImuError::ReadError(format!("Failed to acquire lock on port: {}", e)))?;

        let read_cmd = ReadAddressCommand::new(register).to_bytes();
        port_guard
            .write_all(&read_cmd)
            .map_err(|e| ImuError::WriteError(format!("Failed to write read command: {}", e)))?;

        let mut parser_guard = self.frame_parser.lock().map_err(|e| {
            ImuError::ReadError(format!("Failed to acquire lock on frame parser: {}", e))
        })?;

        std::thread::sleep(Duration::from_millis(5));

        let mut buffer = [0u8; 1024];
        let read_start_time = Instant::now();

        while read_start_time.elapsed() < timeout {
            match port_guard.read(&mut buffer) {
                Ok(n) => {
                    if n > 0 {
                        trace!("Read {} bytes: {:?}", n, &buffer[..n]);
                        match parser_guard.parse(&buffer[0..n]) {
                            Ok(response) => {
                                for frame in response {
                                    match frame {
                                        ReadFrame::GenericRead { data } => {
                                            return Ok(data);
                                        }
                                        _ => {
                                            continue;
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                return Err(ImuError::ReadError(format!(
                                    "Failed to parse response: {}",
                                    e
                                )));
                            }
                        }
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    debug!("Read timed out, continuing wait...");
                }
                Err(e) => {
                    return Err(ImuError::ReadError(format!(
                        "Failed to read from port: {}",
                        e
                    )));
                }
            }
        }

        Err(ImuError::ReadError("Timeout reading register".to_string()))
    }

    pub fn read_all_registers(
        &self,
        timeout_per_register: Duration,
    ) -> Result<Vec<(Register, [u8; 8])>, ImuError> {
        let all_registers = Register::iter();
        let register_count = Register::iter().count();

        let mut results = Vec::with_capacity(register_count);
        let mut errors = Vec::new();

        for register in all_registers {
            match register {
                Register::ReadAddr | Register::Key | Register::Save => {
                    trace!("Skipping ReadAddr, Key, and Save registers: {:?}", register);
                    continue;
                }
                _ => {
                    trace!("Reading register: {:?}", register);
                }
            }

            match self.read_register(register, timeout_per_register) {
                Ok(data) => {
                    results.push((register, data));
                }
                Err(e) => {
                    warn!("Failed to read register {:?}: {}", register, e);
                    errors.push((register, e));
                }
            }
        }

        if errors.is_empty() {
            info!("Successfully read {} registers.", results.len());
            Ok(results)
        } else {
            error!(
                "Failed to read {} registers out of attempted {}.",
                errors.len(),
                register_count - errors.len()
            ); // Adjust count based on skipped
               // For now, returning a generic error indicating partial failure.
            Err(ImuError::ReadError(format!(
                "Failed to read {} registers. First error on {:?}: {}",
                errors.len(),
                errors[0].0,
                errors[0].1
            )))
        }
    }
}

impl ImuReader for HiwonderReader {
    fn stop(&self) -> Result<(), ImuError> {
        self.reset()
    }

    fn get_data(&self) -> Result<ImuData, ImuError> {
        self.data
            .read()
            .map(|data| *data)
            .map_err(|e| ImuError::ReadError(format!("Data lock poisoned: {}", e)))
    }
}

impl Drop for HiwonderReader {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
