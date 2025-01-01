
Link to [IMU](https://www.hiwonder.com/products/imu-module?variant=40375875305559): 

### HiWonder IMU Protocol 
- Data Length: 11 bytes
- Header/start/SOF Byte: 0x55
- Command Byte: 0x51 accel data, 0x52 Gyroscope data, 0x53 angle data
- Data: 8 bytes
- CheckSum Byte: 1 byte (only lowest 8 bits of summation of all bytes in packet is used (`CheckSum & 0xff`))

### Setup
- Install this [driver](https://github.com/WCHSoftGroup/ch341ser_linux) for the CH341 USB controller
- This should create a /dev/ttyUSB0 - you should check which one by doing ls /dev/tty*. You might need to the change the permissions
- Connect the IMU to the computer via USB
- Default Baud Rate: 9600
- Default USB port: /dev/ttyUSB0

