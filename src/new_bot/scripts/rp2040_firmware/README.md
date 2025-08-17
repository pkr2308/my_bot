# RP2040 Firmware for ROS2 Hardware Interface

This directory contains the cleaned-up firmware files for the RP2040 microcontroller used with the ROS2 hardware interface.

## Files Overview

### `main.py` 
**Primary firmware** - Production-ready firmware with:
- Full sensor data simulation (joints, IMU, range sensors)
- Actuator command processing 
- Robust serial communication
- ROS2 hardware interface compatibility
- Non-blocking operation

### `simple_test.py`
**Alternative test firmware** - Simpler version for:
- Basic testing and debugging
- Educational purposes
- Minimal functionality verification

## Quick Start

1. **Flash MicroPython** to your RP2040 (one-time setup)
2. **Upload firmware**: Use `main.py` for production or `simple_test.py` for testing
3. **Test communication**: Verify with test scripts

## Supported Commands

- `PING` → `PONG` - Basic connectivity test
- `INIT` → `OK` - Initialize hardware (LED on)  
- `STOP` → `OK` - Stop hardware (LED off)
- `READ` → `<sensor_data>` - Get all sensor readings
- `CMD,P:pos,V:vel,...` → `OK` - Send actuator commands

## Data Format

**Sensor Data Output**: `<pos0,vel0,pos1,vel1,pos2,vel2,pos3,vel3,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,range0,range1,range2>`

- 4 joints: position and velocity pairs
- IMU: quaternion (qx,qy,qz,qw), gyroscope (gx,gy,gz), accelerometer (ax,ay,az)  
- 3 range sensors: front, left, right distances

## Hardware Compatibility

- **Primary**: Raspberry Pi Pico / RP2040
- **Communication**: USB Serial at 115200 baud
- **LED**: Pin 25 for status indication
- **Expandable**: Add real sensors/actuators as needed

## Integration with ROS2

This firmware is designed to work seamlessly with the ROS2 hardware interface. The data format and command structure match the expected interface protocol.

For setup instructions, see the main `FLASHING_GUIDE.md`.
