# Hexmove

This crate contains the code for interacting with Hexmove IMUs.

Hexmove IMUs operate over a CAN bus. To set up the CAN interface for Hexmove IMUs, run the following command (where can0 is the name of the CAN interface):

```bash
sudo ip link set can0 up type can bitrate 500000
```
