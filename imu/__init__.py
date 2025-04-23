"""Defines the top-level API for the IMU package."""

from .bindings import (
    ImuReader,
    Quaternion,
    Vector3,
    create_bmi088_reader,
    create_bno055_reader,
    create_hexmove_reader,
    create_hiwonder_reader,
)


# Define convenient functions to create different IMU readers
def create_bno055(i2c_device: str = "/dev/i2c-1") -> ImuReader:
    """Create a BNO055 IMU reader on the specified I2C device."""
    return create_bno055_reader(i2c_device)


def create_hiwonder(
    serial_port: str = "/dev/ttyUSB0",
    baud_rate: int = 115200,
    timeout_secs: float = 1.0,
    auto_detect_baud_rate: bool = False,
) -> ImuReader:
    """Create a Hiwonder IMU reader on the specified serial port."""
    return create_hiwonder_reader(serial_port, baud_rate, timeout_secs, auto_detect_baud_rate)


def create_bmi088(i2c_device: str = "/dev/i2c-1") -> ImuReader:
    """Create a BMI088 IMU reader on the specified I2C device."""
    return create_bmi088_reader(i2c_device)


def create_hexmove(can_interface: str = "can0", node_id: int = 1, param_id: int = 1) -> ImuReader:
    """Create a Hexmove IMU reader on the specified CAN interface."""
    return create_hexmove_reader(can_interface, node_id, param_id)


__all__ = [
    # Data types
    "Vector3",
    "Quaternion",
    "ImuReader",
    "create_bno055",
    "create_hiwonder",
    "create_bmi088",
    "create_hexmove",
    "create_bno055_reader",
    "create_hiwonder_reader",
    "create_bmi088_reader",
    "create_hexmove_reader",
]
