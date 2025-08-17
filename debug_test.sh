#!/bin/bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
cd /home/fe25/pranav_bot_ros2_jazzy
source install/setup.bash
timeout 15 ros2 launch new_bot minimal_hardware_test.launch.py
