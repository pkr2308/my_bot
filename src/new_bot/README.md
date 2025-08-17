# New Bot - Ackermann Steering Robot with RP2040

This ROS2 package provides hardware interface and control for an Ackermann steering robot with the following components:

## Hardware Architecture

### Main Components
- **Raspberry Pi 5**: Main computer running ROS2 Jazzy
- **RP2040**: Microcontroller handling low-level hardware control
- **Communication**: RP2040 ↔ RPi5 via Serial/USB

### Actuators (controlled by RP2040)
- **Steering Servo**: Controls front wheel steering angle
- **DC Motor**: Single motor driving rear wheels via TB6612 motor driver

### Sensors (connected to RP2040)
- **BNO055 IMU**: 9-axis IMU via I2C
- **3x TF Luna Lidars**: Front, left, and right distance sensors via I2C

### Direct RPi5 Sensors
- **RPLidar A1**: 2D laser scanner via USB
- **PiCamera 3**: Wide-angle camera via CSI

## Package Structure

```
new_bot/
├── config/
│   ├── controllers.yaml          # ROS2 controller configuration
│   └── robot_view.rviz          # RViz visualization config
├── description/
│   ├── robot.urdf.xacro         # Main robot description
│   ├── robot_core.xacro         # Robot physical structure
│   ├── ros2_control.xacro       # Hardware interface configuration
│   ├── lidar.xacro             # Lidar mount description
│   └── inertial_macros.xacro   # Inertial properties macros
├── launch/
│   ├── hardware.launch.py       # Main hardware launch file
│   └── robot_state_publisher.launch.py
├── scripts/
│   └── test_rp2040.py          # RP2040 communication test script
├── src/
│   └── rp2040_hardware_interface.cpp  # Custom hardware interface
├── include/new_bot/
│   └── rp2040_hardware_interface.hpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies

### System Dependencies
```bash
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-ackermann-steering-controller \
    ros-jazzy-imu-sensor-broadcaster \
    ros-jazzy-range-sensor-broadcaster \
    ros-jazzy-robot-localization \
    ros-jazzy-rplidar-ros \
    ros-jazzy-v4l2-camera \
    python3-serial
```

### Hardware Setup
1. **Serial Permissions**: Add user to dialout group
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```

2. **Device Permissions**: Ensure devices are accessible
   ```bash
   # RP2040 (adjust based on your device)
   ls -la /dev/ttyACM*
   
   # RPLidar
   ls -la /dev/ttyUSB*
   
   # Camera
   ls -la /dev/video*
   ```

## Building the Package

```bash
cd ~/your_ros2_workspace
colcon build --packages-select new_bot
source install/setup.bash
```

## Usage

### 1. Test RP2040 Communication
Before running ROS2, test that communication with RP2040 works:

```bash
cd ~/your_ros2_workspace
source install/setup.bash
python3 src/new_bot/scripts/test_rp2040.py --port /dev/ttyACM0 --baud 115200
```

### 2. Launch Hardware Interface
```bash
ros2 launch new_bot hardware.launch.py
```

Optional parameters:
- `serial_port:=/dev/ttyACM0` - RP2040 serial port
- `rviz:=true` - Start RViz visualization
- `use_sim_time:=false` - Use system time (not simulation time)

### 3. Control the Robot

#### Send velocity commands:
```bash
ros2 topic pub /ackermann_controller/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.5}}' --once
```

#### Check joint states:
```bash
ros2 topic echo /joint_states
```

#### Monitor IMU data:
```bash
ros2 topic echo /imu/data
```

#### Check range sensors:
```bash
ros2 topic echo /tf_luna_front/range
ros2 topic echo /tf_luna_left/range
ros2 topic echo /tf_luna_right/range
```

### 4. View in RViz
RViz should start automatically with the hardware launch. If not:
```bash
ros2 run rviz2 rviz2 -d src/new_bot/config/robot_view.rviz
```

## RP2040 Firmware Requirements

Your RP2040 firmware should implement the following serial communication protocol:

### Command Format
Commands are sent as: `<COMMAND>\n`

### Required Commands
- `<PING>` → Response: `<PONG>`
- `<INIT>` → Initialize hardware
- `<READ>` → Return sensor data
- `<CMD,P:pos1,P:pos2,V:vel1,V:vel2>` → Set actuator commands
- `<STOP>` → Emergency stop

### Sensor Data Format
Response to `<READ>` command:
```
<pos1,vel1,pos2,vel2,pos3,vel3,pos4,vel4,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,range_f,range_l,range_r>
```

Where:
- `pos1,vel1` to `pos4,vel4`: Joint positions (rad) and velocities (rad/s)
- `qx,qy,qz,qw`: IMU quaternion orientation
- `gx,gy,gz`: IMU angular velocities (rad/s)
- `ax,ay,az`: IMU linear accelerations (m/s²)
- `range_f,range_l,range_r`: TF Luna distances (m)

## Troubleshooting

### Common Issues

1. **Serial connection failed**
   - Check device permissions and port
   - Verify RP2040 is connected and running firmware
   - Check baud rate matches

2. **Controllers not starting**
   - Check that hardware interface is loaded
   - Verify ros2_control configuration
   - Check joint names match between URDF and controllers

3. **No sensor data**
   - Test RP2040 communication with test script
   - Check sensor wiring and I2C addresses
   - Verify firmware sensor reading functions

4. **Robot doesn't move**
   - Check actuator connections
   - Verify command format and ranges
   - Test individual components with RP2040 test script

### Debug Commands

```bash
# Check available controllers
ros2 control list_controllers

# Check hardware interface
ros2 control list_hardware_interfaces

# Monitor topics
ros2 topic list
ros2 topic hz /joint_states

# Check transforms
ros2 run tf2_tools view_frames.py
```

## Next Steps for SLAM and Navigation

Once hardware interface is working:

1. **Calibrate sensors** - IMU and wheel odometry
2. **Install navigation stack**:
   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
   ```
3. **Create maps** using SLAM
4. **Configure navigation** parameters
5. **Add autonomous navigation** capabilities

## Contributing

When modifying this package:
1. Test changes with `test_rp2040.py` first
2. Update URDF if physical changes are made
3. Adjust controller parameters as needed
4. Update this README with new features
