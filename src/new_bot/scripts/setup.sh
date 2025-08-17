#!/bin/bash

# New Bot Setup Script
# Run this script to set up dependencies and permissions for the new_bot package

echo "========================================="
echo "New Bot Hardware Setup"
echo "========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check ROS2 package
check_ros_package() {
    if ros2 pkg list | grep -q "^$1$"; then
        echo -e "${GREEN}✓${NC} $1 is installed"
        return 0
    else
        echo -e "${RED}✗${NC} $1 is not installed"
        return 1
    fi
}

echo "Checking system dependencies..."

# Check if ROS2 is installed
if command_exists ros2; then
    echo -e "${GREEN}✓${NC} ROS2 is installed"
else
    echo -e "${RED}✗${NC} ROS2 is not installed"
    exit 1
fi

# Check Python3 and pip
if command_exists python3; then
    echo -e "${GREEN}✓${NC} Python3 is installed"
else
    echo -e "${RED}✗${NC} Python3 is not installed"
fi

echo ""
echo "Checking ROS2 packages..."

# List of required packages
packages=(
    "ros2_control"
    "ros2_controllers" 
    "controller_manager"
    "joint_state_broadcaster"
    "ackermann_steering_controller"
    "imu_sensor_broadcaster"
    "robot_state_publisher"
    "robot_localization"
    "rplidar_ros"
)

missing_packages=()

for package in "${packages[@]}"; do
    if ! check_ros_package "$package"; then
        missing_packages+=("$package")
    fi
done

echo ""
echo "Checking hardware devices..."

# Check serial devices
if [ -e "/dev/ttyACM0" ]; then
    echo -e "${GREEN}✓${NC} /dev/ttyACM0 exists (likely RP2040)"
elif [ -e "/dev/ttyUSB0" ]; then
    echo -e "${YELLOW}!${NC} /dev/ttyUSB0 exists (check if this is your RP2040)"
else
    echo -e "${RED}✗${NC} No serial devices found (/dev/ttyACM* or /dev/ttyUSB*)"
    echo "  Connect your RP2040 and try again"
fi

# Check user permissions
if groups $USER | grep -q dialout; then
    echo -e "${GREEN}✓${NC} User is in dialout group"
else
    echo -e "${RED}✗${NC} User not in dialout group"
    echo "  Run: sudo usermod -a -G dialout $USER"
    echo "  Then logout and login again"
fi

# Check for lidar
if ls /dev/ttyUSB* >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} USB serial devices found (likely RPLidar)"
else
    echo -e "${YELLOW}!${NC} No USB serial devices found for RPLidar"
fi

# Check for camera
if [ -e "/dev/video0" ]; then
    echo -e "${GREEN}✓${NC} Camera device found at /dev/video0"
else
    echo -e "${YELLOW}!${NC} No camera found at /dev/video0"
fi

echo ""
echo "========================================="

# Install missing packages
if [ ${#missing_packages[@]} -gt 0 ]; then
    echo -e "${YELLOW}Installing missing ROS2 packages...${NC}"
    
    install_cmd="sudo apt update && sudo apt install -y"
    for package in "${missing_packages[@]}"; do
        install_cmd="$install_cmd ros-jazzy-${package//_/-}"
    done
    
    # Add additional system packages
    install_cmd="$install_cmd python3-serial libserial-dev"
    
    echo "Running: $install_cmd"
    eval $install_cmd
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} Packages installed successfully"
    else
        echo -e "${RED}✗${NC} Failed to install some packages"
    fi
else
    echo -e "${GREEN}✓${NC} All required packages are installed"
fi

echo ""
echo "Building the new_bot package..."

# Build the package
cd "$(dirname "$0")/../../../.."  # Go to workspace root
colcon build --packages-select new_bot

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Package built successfully"
else
    echo -e "${RED}✗${NC} Package build failed"
    exit 1
fi

echo ""
echo "========================================="
echo "Setup Summary:"
echo "========================================="

if [ ${#missing_packages[@]} -eq 0 ]; then
    echo -e "${GREEN}✓${NC} All dependencies satisfied"
else
    echo -e "${YELLOW}!${NC} Some packages were missing but should now be installed"
fi

echo ""
echo "Next Steps:"
echo "1. Flash your RP2040 with the appropriate firmware"
echo "2. Connect all hardware (RP2040, RPLidar, Camera)"
echo "3. Test communication: python3 src/new_bot/scripts/test_rp2040.py"
echo "4. Launch the robot: ros2 launch new_bot hardware.launch.py"
echo ""
echo "For detailed instructions, see: src/new_bot/README.md"
echo ""
echo -e "${GREEN}Setup complete!${NC}"

echo ""
echo "========================================="
echo "RP2040 Development Tools Setup"
echo "========================================="

# Check for RP2040 development tools
echo "Checking RP2040 development environment..."

# Check if Pico SDK is installed
PICO_SDK_PATH="$HOME/pico-sdk"
if [ -d "$PICO_SDK_PATH" ]; then
    echo -e "${GREEN}✓${NC} Pico SDK found at $PICO_SDK_PATH"
else
    echo -e "${YELLOW}!${NC} Pico SDK not found, installing..."
    cd "$HOME"
    git clone -b master https://github.com/raspberrypi/pico-sdk.git
    cd pico-sdk
    git submodule update --init
    echo -e "${GREEN}✓${NC} Pico SDK installed"
fi

# Set up environment variable
if ! grep -q "PICO_SDK_PATH" ~/.bashrc; then
    echo -e "${YELLOW}!${NC} Adding PICO_SDK_PATH to ~/.bashrc"
    echo "export PICO_SDK_PATH=$PICO_SDK_PATH" >> ~/.bashrc
    echo -e "${GREEN}✓${NC} Environment variable added"
else
    echo -e "${GREEN}✓${NC} PICO_SDK_PATH already in ~/.bashrc"
fi

export PICO_SDK_PATH="$PICO_SDK_PATH"

# Check for ARM toolchain
if command_exists arm-none-eabi-gcc; then
    echo -e "${GREEN}✓${NC} ARM GCC toolchain is installed"
else
    echo -e "${YELLOW}!${NC} Installing ARM GCC toolchain..."
    sudo apt install -y gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
fi

# Create build directory for RP2040 firmware
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
BUILD_DIR="$SCRIPT_DIR/pico_build"

echo -e "${YELLOW}!${NC} Setting up RP2040 firmware build directory..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Create CMakeLists.txt for RP2040 firmware
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.13)

# Import the Pico SDK
include(pico_sdk_import.cmake)

project(rp2040_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable
add_executable(rp2040_firmware
    ../rp2040_firmware.cpp
)

# Link libraries
target_link_libraries(rp2040_firmware 
    pico_stdlib 
    hardware_pwm 
    hardware_i2c 
    hardware_uart
    hardware_timer
)

# Enable USB serial, disable UART serial
pico_enable_stdio_usb(rp2040_firmware 1)
pico_enable_stdio_uart(rp2040_firmware 0)

# Create additional outputs (UF2, hex, etc.)
pico_add_extra_outputs(rp2040_firmware)
EOF

# Copy pico_sdk_import.cmake
if [ -f "$PICO_SDK_PATH/external/pico_sdk_import.cmake" ]; then
    cp "$PICO_SDK_PATH/external/pico_sdk_import.cmake" .
    echo -e "${GREEN}✓${NC} CMake files prepared"
else
    echo -e "${RED}✗${NC} Could not find pico_sdk_import.cmake"
    echo "    Make sure Pico SDK is properly installed"
fi

# Build the firmware if cmake files are ready
if [ -f "pico_sdk_import.cmake" ]; then
    echo -e "${YELLOW}!${NC} Building RP2040 firmware..."
    cmake . && make -j$(nproc)
    
    if [ -f "rp2040_firmware.uf2" ]; then
        echo -e "${GREEN}✓${NC} RP2040 firmware built successfully!"
        echo ""
        echo "📋 To flash the firmware:"
        echo "1. Hold BOOTSEL button on your RP2040 while plugging it in"
        echo "2. Copy the firmware:"
        echo "   cp $BUILD_DIR/rp2040_firmware.uf2 /media/\$USER/RPI-RP2/"
        echo "3. The RP2040 will reboot and run the new firmware"
    else
        echo -e "${YELLOW}!${NC} RP2040 firmware build failed, but files are prepared"
        echo "    You can build manually with: cd $BUILD_DIR && cmake . && make"
    fi
fi

echo ""
