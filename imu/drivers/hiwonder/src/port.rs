
use serialport::SerialPort;

pub struct ImuConfigPort {
    serial_port: Box<dyn SerialPort>,
}

impl ImuConfigPort {
    pub fn new(port_name: &str, baud_rate: u32) -> Result<Self, serialport::Error> {
        let serial_port = serialport::new(port_name, baud_rate)
            .timeout(std::time::Duration::from_secs(2))
            .open()?;
        // clear the serial port buffer
        let mut ret = Self { serial_port };
        println!("Opened port: {}", port_name);
        ret.clear(std::time::Duration::from_millis(10))?;
        Ok(ret)
    }

    pub fn verify_comms(
        &mut self,
        timeout: std::time::Duration,
    ) -> Result<(), serialport::Error> {

        // read 44 bytes from the serial port
        let mut buffer = [0u8; 44];
        self.read_bytes(&mut buffer, timeout)?;

        // scan for the 0x55 byte
        let idx = buffer.iter().position(|&x| x == 0x55).ok_or(serialport::Error::new(
            serialport::ErrorKind::Io(std::io::ErrorKind::InvalidData),
            "0x55 not found in buffer",
        ))?;

        // calculate checksum starting at idx
        if Self::checksum(&buffer[idx..idx + 11]) != buffer[idx + 10] {
            return Err(serialport::Error::new(
                serialport::ErrorKind::Io(std::io::ErrorKind::InvalidData),
                "Checksum mismatch",
            ));
        }
        // checksum matched, we are good!
        Ok(())
    }

    // find baud rate by iterating over the common baud rates
    pub fn find_valid_baud_rate(
        &mut self,
        baud_rates: &[u32],
        timeout: std::time::Duration,
    ) -> Result<u32, serialport::Error> {

        for &baud_rate in baud_rates {
            if self.try_baud_rate(baud_rate, timeout).is_ok() {
                return Ok(baud_rate);
            }
        }
        Err(serialport::Error::new(
            serialport::ErrorKind::Io(std::io::ErrorKind::InvalidData),
            "No valid baud rate found",
        ))
    }

    // TODO verify that u8 is at least 11 bytes long
    pub fn checksum(buffer: &[u8]) -> u8 {
        // iterate over the first 10 bytes, reduce them to sum, and get the lowest byte
        (buffer.iter().take(10).map(|&x| x as u16).sum::<u16>() & 0xff) as u8
    }

    pub fn try_baud_rate(
        &mut self,
        baud_rate: u32,
        timeout: std::time::Duration,
    ) -> Result<(), serialport::Error> {
        // set baud rate and verify comms
        self.set_baud_rate(baud_rate)?;
        println!("Trying baud rate: {}", baud_rate);
        self.verify_comms(timeout)
    }

    fn clear(&mut self, timeout: std::time::Duration) -> Result<(), serialport::Error> {
        // clear the serial port buffer by reading a large 
        let mut buffer = [0u8; 64];
        self.serial_port.set_timeout(timeout)?;
        for i in 0..4 {
            // read 64 bytes from the serial port
            match self.serial_port.read(&mut buffer) {
                Ok(bytes_read) => {
                    // println!("Cleared {} bytes from port", bytes_read);
                    if bytes_read == 0 {
                        break;
                    }
                }
                Err(e) => {
                    // println!("Error clearing port: {}", e);
                }
            }
        }
        Ok(())
    }

    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<(), serialport::Error> {
        self.serial_port.set_baud_rate(baud_rate)?;
        // sleep for 200ms to allow the port to reset
        std::thread::sleep(std::time::Duration::from_millis(10));
        // read all stale data from the port
        self.clear(std::time::Duration::from_millis(10))
    }

    pub fn read_bytes(
        &mut self,
        buffer: &mut [u8],
        timeout: std::time::Duration,
    ) -> Result<usize, serialport::Error> {
        self.serial_port.set_timeout(timeout)?;
        let bytes_read = self.serial_port.read(buffer)?;
        Ok(bytes_read)
    }

    pub fn write_bytes(
        &mut self,
        data: &[u8],
        timeout: std::time::Duration,
    ) -> Result<(), serialport::Error> {
        self.serial_port.set_timeout(timeout)?;
        self.serial_port.write(data)?;
        Ok(())
    }
}

