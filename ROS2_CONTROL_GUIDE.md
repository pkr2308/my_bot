# ROS2 Robot Control Quick Reference

## Starting the Robot System

### Full Robot Launch (Recommended)
```bash
cd /home/fe25/pranav_bot_ros2_jazzy
source install/setup.bash
ros2 launch new_bot hardware.launch.py
```

### Manual Component Launch
```bash
# 1. Start robot description
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro src/new_bot/description/robot.urdf.xacro)"

# 2. Start hardware interface (in new terminal)
source install/setup.bash
ros2 run controller_manager ros2_control_node --ros-args --params-file src/new_bot/config/controllers.yaml

# 3. Load controllers (in new terminal) 
source install/setup.bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner ackermann_controller
ros2 run controller_manager spawner imu_broadcaster
```

## Robot Control Commands

### Basic Movement
```bash
# Drive forward
ros2 topic pub -1 /ackermann_controller/cmd_ackermann ackermann_msgs/AckermannDriveStamped '{drive: {speed: 1.0, steering_angle: 0.0}}'

# Turn left while moving
ros2 topic pub -1 /ackermann_controller/cmd_ackermann ackermann_msgs/AckermannDriveStamped '{drive: {speed: 0.5, steering_angle: 0.5}}'

# Stop
ros2 topic pub -1 /ackermann_controller/cmd_ackermann ackermann_msgs/AckermannDriveStamped '{drive: {speed: 0.0, steering_angle: 0.0}}'
```

### Monitoring Robot State

```bash
# View all available topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Monitor IMU data
ros2 topic echo /imu_broadcaster/imu

# Monitor range sensors  
ros2 topic echo /range_sensor_broadcaster/range

# Check controller status
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### Control System Status

```bash
# Check if hardware interface is active
ros2 control list_hardware_interfaces

# Expected output:
# new_bot_hardware[new_bot/RP2040HardwareInterface] - active

# Check controller states
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# ackermann_controller[ackermann_steering_controller/AckermannSteeringController] active  
# imu_broadcaster[imu_sensor_broadcaster/ImuSensorBroadcaster] active
```

## Troubleshooting

### Hardware Interface Issues
```bash
# Check RP2040 connection
ls -la /dev/ttyACM*

# Test RP2040 directly
python3 src/new_bot/scripts/improved_test_rp2040.py /dev/ttyACM0

# Check ROS2 logs
ros2 log set logger_level RP2040HardwareInterface DEBUG
```

### Controller Issues  
```bash
# Restart a controller
ros2 control switch_controllers --stop ackermann_controller
ros2 control switch_controllers --start ackermann_controller

# Check controller configuration
ros2 param list /controller_manager
```

### Data Flow Verification
```bash
# 1. Verify robot description is published
ros2 topic echo /robot_description --once

# 2. Verify hardware interface is reading data  
ros2 topic echo /joint_states

# 3. Verify controllers are processing commands
ros2 topic pub /ackermann_controller/cmd_ackermann ackermann_msgs/AckermannDriveStamped '{drive: {speed: 0.1, steering_angle: 0.0}}'
```

## Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   ROS2 Nodes    │───→│ Controller Mgr   │───→│ Hardware Interface│
│ (Your Code)     │    │ - Joint State    │    │ (RP2040Interface) │
│                 │    │ - Ackermann      │    │                  │
│                 │    │ - IMU Broadcast  │    │                  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                         │
                                                    Serial (LibSerial)
                                                         │
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ Physical Robot  │←───│  RP2040 MCU      │←───│  MicroPython    │
│ - Motors        │    │  - GPIO Control  │    │  Firmware       │
│ - Sensors       │    │  - ADC Reading   │    │  (main.py)      │
│ - Actuators     │    │  - PWM Output    │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Integration Complete! 🎉

Your robot now has:
- ✅ **ROS2 Control Integration**: Standard ROS2 controllers
- ✅ **Real-time Communication**: Hardware interface ↔ RP2040  
- ✅ **Sensor Data Streaming**: IMU, joint states, range sensors
- ✅ **Motor Control**: Position and velocity commands
- ✅ **Standard Interface**: Works with existing ROS2 tools

**Ready for autonomous navigation, mapping, and advanced robotics applications!**
