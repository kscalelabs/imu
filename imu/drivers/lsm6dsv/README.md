# LSM6DSV Driver

SPI driver for the LSM6DSV 6-axis IMU sensor from STMicroelectronics.

## Features

- Accelerometer and gyroscope data reading
- SPI communication interface
- Configurable output data rates and full-scale ranges
- Temperature reading
- Background thread for continuous data acquisition

## Usage

```rust
use lsm6dsv::Lsm6dsvReader;

let reader = Lsm6dsvReader::new("/dev/spidev1.0")?;
let data = reader.get_data()?;
```

