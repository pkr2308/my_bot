# RP2040 Hardware Interface

This directory contains the RP2040 microcontroller firmware and testing tools for the ROS2 hardware interface.

## Files

- `rp2040_firmware.cpp` - C++ firmware for RP2040 microcontroller
- `rp2040_simulator.py` - Python simulator for testing without hardware
- `test_rp2040.py` - Test script for communication validation
- `setup.sh` - Setup script for development environment

## Hardware Setup

### RP2040 Connections

```
Pin Assignment:
- GPIO 0:  Servo PWM (steering)
- GPIO 1:  Motor PWM (drive)
- GPIO 2:  Motor Direction
- GPIO 4:  I2C SDA (IMU)
- GPIO 5:  I2C SCL (IMU)
- GPIO 8:  UART TX (TF-Luna sensors)
- GPIO 9:  UART RX (TF-Luna sensors)
- GPIO 25: Status LED (built-in)
```

### Required Hardware

1. **RP2040 Development Board** (Raspberry Pi Pico or similar)
2. **Servo Motor** - For Ackermann steering
3. **DC Motor Driver** - L298N or similar for drive motor
4. **IMU Sensor** - MPU6050, BNO055, or similar (I2C)
5. **TF-Luna LiDAR Sensors** (3x) - For range sensing (UART)
6. **USB Cable** - For communication with main computer

## Software Setup

### 1. Install RP2040 Development Tools

```bash
# Install Pico SDK
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# Set environment variable
export PICO_SDK_PATH=~/pico-sdk
echo 'export PICO_SDK_PATH=~/pico-sdk' >> ~/.bashrc

# Install build tools
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

### 2. Build RP2040 Firmware

```bash
# Create build directory
cd /path/to/your/project/scripts
mkdir pico_build
cd pico_build

# Create CMakeLists.txt for Pico
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(rp2040_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040_firmware
    ../rp2040_firmware.cpp
)

target_link_libraries(rp2040_firmware 
    pico_stdlib 
    hardware_pwm 
    hardware_i2c 
    hardware_uart
    hardware_timer
)

pico_enable_stdio_usb(rp2040_firmware 1)
pico_enable_stdio_uart(rp2040_firmware 0)
pico_add_extra_outputs(rp2040_firmware)
EOF

# Copy pico_sdk_import.cmake
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .

# Build firmware
cmake ..
make -j4
```

### 3. Flash Firmware to RP2040

1. Hold BOOTSEL button while plugging in the RP2040
2. Copy the generated `.uf2` file to the mounted drive:
   ```bash
   cp rp2040_firmware.uf2 /media/$USER/RPI-RP2/
   ```
3. The RP2040 will automatically reboot and start running the firmware

## Testing

### 1. Test with Simulator (No Hardware Required)

```bash
# Run interactive simulator
python3 rp2040_simulator.py --interactive

# In the simulator prompt, try these commands:
RP2040> PING
RP2040> INIT
RP2040> READ
RP2040> CMD,P:0.5,P:0.5,V:0.0,V:0.0
RP2040> STOP
RP2040> quit
```

### 2. Test with Real Hardware

First, find your RP2040 device:
```bash
# List available serial ports
ls /dev/ttyACM* /dev/ttyUSB*

# Check permissions (add your user to dialout group if needed)
sudo usermod -a -G dialout $USER
# Log out and back in for group changes to take effect
```

Run the test script:
```bash
# Test with default port
python3 test_rp2040.py

# Test with specific port
python3 test_rp2040.py --port /dev/ttyACM0 --baud 115200
```

### 3. Test with ROS2 Hardware Interface

```bash
# Build your ROS2 package
cd /path/to/your/ros2_workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Launch the hardware interface
ros2 launch new_bot hardware.launch.py
```

## Communication Protocol

### Command Format
All commands are wrapped with `<>` markers:
- `<PING>` - Test communication (response: `<PONG>`)
- `<INIT>` - Initialize system (response: `<OK>`)
- `<READ>` - Get sensor data (response: sensor data)
- `<CMD,P:pos1,P:pos2,V:vel1,V:vel2>` - Send control commands
- `<STOP>` - Stop all motion (response: `<OK>`)

### Sensor Data Format
Response to `READ` command:
```
<joint_pos_0,joint_vel_0,joint_pos_1,joint_vel_1,joint_pos_2,joint_vel_2,joint_pos_3,joint_vel_3,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,range_front,range_left,range_right>
```

Where:
- `joint_pos/vel_X` - Joint positions and velocities
- `qx,qy,qz,qw` - IMU quaternion orientation
- `gx,gy,gz` - IMU angular velocity (rad/s)
- `ax,ay,az` - IMU linear acceleration (m/s²)
- `range_X` - Distance sensor readings (meters)

## Troubleshooting

### Common Issues

1. **Permission Denied on Serial Port**
   ```bash
   sudo usermod -a -G dialout $USER
   # Then log out and back in
   ```

2. **RP2040 Not Detected**
   - Check USB connection
   - Try different USB port/cable
   - Re-flash firmware in BOOTSEL mode

3. **Communication Timeout**
   - Verify baud rate (115200)
   - Check serial port path
   - Ensure firmware is running (LED should be on)

4. **Compilation Errors**
   - Verify PICO_SDK_PATH is set correctly
   - Install missing dependencies
   - Check CMake version (≥3.13 required)

### Debug Output

The RP2040 firmware outputs debug information over USB serial. You can view it with:
```bash
# Using minicom
sudo apt install minicom
minicom -D /dev/ttyACM0 -b 115200

# Using screen
screen /dev/ttyACM0 115200

# Using Python
python3 -c "
import serial
ser = serial.Serial('/dev/ttyACM0', 115200)
while True:
    print(ser.readline().decode(), end='')
"
```

## Next Steps

1. **Add Real Sensors**: Replace simulated sensor readings with actual hardware
2. **Tune Control**: Adjust PID parameters for servo and motor control
3. **Safety Features**: Add emergency stops and fault detection
4. **Calibration**: Implement sensor calibration routines
5. **Performance**: Optimize communication frequency and latency
