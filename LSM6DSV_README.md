# LSM6DSV IMU Driver Implementation

This document describes the implementation of the LSM6DSV 6-axis IMU driver for the IMU Python package.

## Overview

The LSM6DSV is a 6-axis IMU (Inertial Measurement Unit) from STMicroelectronics that combines a 3-axis accelerometer and 3-axis gyroscope. This implementation provides SPI communication support for reading sensor data.

## Features

- **SPI Communication**: Full SPI interface support with configurable settings
- **Accelerometer**: 3-axis acceleration data with configurable ranges (±2g, ±4g, ±8g, ±16g)
- **Gyroscope**: 3-axis angular velocity data with configurable ranges (±125 to ±4000 dps)
- **Temperature**: Built-in temperature sensor
- **Background Threading**: Continuous data acquisition in background thread
- **Configurable ODR**: Adjustable output data rates for both sensors
- **Python Integration**: Seamless integration with the existing IMU package

## Hardware Requirements

- LSM6DSV sensor connected via SPI
- Linux system with SPI support
- Appropriate permissions to access SPI devices (typically `/dev/spidev*`)

## SPI Connection

The LSM6DSV uses SPI mode 0 (CPOL=0, CPHA=0) with the following connections:

- **MOSI**: Master Out Slave In
- **MISO**: Master In Slave Out  
- **SCLK**: Serial Clock
- **CS**: Chip Select (active low)
- **VCC**: 1.71V to 3.6V
- **GND**: Ground

## Usage

### Basic Usage

```python
from imu import create_lsm6dsvtr

# Create an LSM6DSV reader
reader = create_lsm6dsvtr("/dev/spidev1.0")

# Read sensor data
data = reader.get_data()

if data.get('accelerometer'):
    accel = data['accelerometer']
    print(f"Acceleration: x={accel.x:.3f}, y={accel.y:.3f}, z={accel.z:.3f} m/s²")

if data.get('gyroscope'):
    gyro = data['gyroscope']
    print(f"Angular velocity: x={gyro.x:.3f}, y={gyro.y:.3f}, z={gyro.z:.3f} deg/s")

if data.get('temperature') is not None:
    print(f"Temperature: {data['temperature']:.1f}°C")

# Clean up
reader.stop()
```

### Advanced Configuration

The driver supports runtime configuration of sensor ranges and output data rates:

```python
# Note: These methods would need to be exposed in the Python bindings
# reader.set_accel_range(AccelRange.G4)  # ±4g range
# reader.set_gyro_range(GyroRange.Dps500)  # ±500 dps range
# reader.set_accel_odr(AccelOdr.Hz208)  # 208 Hz output rate
# reader.set_gyro_odr(GyroOdr.Hz208)  # 208 Hz output rate
```

## Implementation Details

### File Structure

```
imu/drivers/lsm6dsv/
├── Cargo.toml          # Rust package configuration
├── README.md           # Driver-specific documentation
└── src/
    ├── lib.rs          # Main driver implementation
    ├── registers.rs    # Register definitions and constants
    └── spi.rs          # SPI communication layer
```

### Key Components

1. **Register Definitions** (`registers.rs`):
   - All LSM6DSV register addresses
   - Output data rate (ODR) settings
   - Full-scale range settings
   - Sensitivity constants

2. **SPI Communication** (`spi.rs`):
   - Low-level SPI read/write operations
   - Register-based communication
   - Error handling for SPI operations

3. **Main Driver** (`lib.rs`):
   - High-level sensor interface
   - Data conversion and scaling
   - Background thread management
   - Configuration commands

### Data Conversion

- **Accelerometer**: Raw values are converted from mg to m/s² using appropriate sensitivity factors
- **Gyroscope**: Raw values are converted from mdps to deg/s using appropriate sensitivity factors  
- **Temperature**: Raw values are converted to Celsius using the formula: `temp = (raw * 1/256) + 25`

### Error Handling

The driver provides comprehensive error handling for:
- SPI communication failures
- Invalid chip ID detection
- Register read/write errors
- Thread synchronization issues

## Testing

A test script is provided (`test_lsm6dsv.py`) that can be used to verify the implementation:

```bash
python3 test_lsm6dsv.py
```

This script will:
1. Test import functionality
2. Attempt to create an LSM6DSV reader
3. Try to read sensor data
4. Test cleanup procedures

## Dependencies

- `spidev`: Linux SPI device interface
- `byteorder`: Byte order conversion utilities
- `log`: Logging framework
- `imu-traits`: Common IMU interface traits

## Limitations

- Linux-only support (SPI device access)
- Requires appropriate permissions for SPI device access
- No magnetometer support (LSM6DSV doesn't include a magnetometer)
- No quaternion/euler angle calculation (accelerometer + gyroscope only)

## Future Enhancements

Potential improvements could include:
- I2C communication support
- Advanced filtering and sensor fusion
- Interrupt-driven data acquisition
- Additional sensor features (if available in LSM6DSV variants)

## Troubleshooting

### Common Issues

1. **Permission Denied**: Ensure the user has access to SPI devices
   ```bash
   sudo usermod -a -G spi $USER
   ```

2. **Device Not Found**: Verify the SPI device path exists
   ```bash
   ls -la /dev/spidev*
   ```

3. **Invalid Chip ID**: Check SPI connections and power supply

4. **No Data**: Verify sensor initialization and SPI communication

### Debug Mode

Enable debug logging to troubleshoot issues:
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

