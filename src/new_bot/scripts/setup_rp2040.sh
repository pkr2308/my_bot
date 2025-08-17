#!/bin/bash

# RP2040 Setup Script for ROS2 Hardware Interface
# This script helps you flash and test the RP2040 firmware

set -e

echo "=== RP2040 Setup for ROS2 Hardware Interface ==="
echo

# Check if RP2040 is connected
check_rp2040() {
    echo "Checking for RP2040 device..."
    
    if ls /dev/ttyACM* >/dev/null 2>&1; then
        echo "✓ Found RP2040 at: $(ls /dev/ttyACM*)"
        return 0
    elif ls /dev/ttyUSB* >/dev/null 2>&1; then
        echo "✓ Found device at: $(ls /dev/ttyUSB*)"
        return 0
    else
        echo "✗ No RP2040 device found"
        echo "  Make sure RP2040 is connected via USB"
        return 1
    fi
}

# Check if RP2040 is in bootloader mode
check_bootloader() {
    echo "Checking for RP2040 in bootloader mode..."
    
    # Check USB devices
    if lsusb | grep -q "2e8a:0003"; then
        echo "✓ RP2040 detected in bootloader mode (USB Mass Storage)"
        return 0
    fi
    
    # Check for mounted RPI-RP2 drive in various locations
    for mount_point in "/media/$USER/RPI-RP2" "/media/RPI-RP2" "/mnt/RPI-RP2"; do
        if [ -d "$mount_point" ]; then
            echo "✓ RPI-RP2 drive found at: $mount_point"
            return 0
        fi
    done
    
    # Check if it's mounted somewhere else
    if mount | grep -q "RPI-RP2"; then
        local mount_path=$(mount | grep "RPI-RP2" | awk '{print $3}')
        echo "✓ RPI-RP2 drive found at: $mount_path"
        return 0
    fi
    
    # Check block devices
    if lsblk | grep -q "RPI-RP2"; then
        echo "✓ RPI-RP2 device detected but may not be mounted"
        echo "Available block devices:"
        lsblk | grep -A1 -B1 "RPI-RP2"
        return 0
    fi
    
    echo "✗ RP2040 not found in bootloader mode"
    return 1
}

# Find the RPI-RP2 mount point
find_rpi_mount() {
    # Try common mount points
    for mount_point in "/media/$USER/RPI-RP2" "/media/RPI-RP2" "/mnt/RPI-RP2"; do
        if [ -d "$mount_point" ]; then
            echo "$mount_point"
            return 0
        fi
    done
    
    # Check mount command
    local mount_path=$(mount | grep "RPI-RP2" | awk '{print $3}' | head -1)
    if [ -n "$mount_path" ]; then
        echo "$mount_path"
        return 0
    fi
    
    return 1
}

# Flash MicroPython firmware
flash_micropython() {
    echo
    echo "Step 1: Flash MicroPython firmware"
    echo "Follow these steps to put RP2040 in bootloader mode:"
    echo "1. Disconnect your RP2040 from USB"
    echo "2. Hold down the BOOTSEL button on the RP2040"
    echo "3. Connect it to USB while holding BOOTSEL"
    echo "4. Release the BOOTSEL button"
    echo "5. The RP2040 should appear as RPI-RP2 drive"
    echo
    
    echo "Checking current USB devices..."
    lsusb | grep -E "(2e8a|Raspberry|Pico)" || echo "  No RP2040 found in USB devices"
    echo
    
    read -p "Press Enter when you've completed the bootloader steps..."
    
    echo "Checking for bootloader mode..."
    if check_bootloader; then
        # Find the mount point
        RPI_MOUNT=$(find_rpi_mount)
        if [ $? -eq 0 ]; then
            echo "Using mount point: $RPI_MOUNT"
        else
            # Try to mount if it's not mounted
            echo "Trying to mount RPI-RP2 device..."
            
            # Create mount point if needed
            sudo mkdir -p /mnt/RPI-RP2
            
            # Try to find the device
            RPI_DEVICE=$(lsblk -o NAME,LABEL | grep "RPI-RP2" | awk '{print "/dev/"$1}' | head -1)
            
            if [ -n "$RPI_DEVICE" ]; then
                sudo mount "$RPI_DEVICE" /mnt/RPI-RP2
                RPI_MOUNT="/mnt/RPI-RP2"
                echo "✓ Mounted $RPI_DEVICE at $RPI_MOUNT"
            else
                echo "✗ Cannot find RPI-RP2 device to mount"
                echo "Available block devices:"
                lsblk
                echo
                echo "Please mount the RPI-RP2 device manually or check your connection"
                return 1
            fi
        fi
        
        echo "Copying MicroPython firmware..."
        
        if [ -f "$HOME/rp2040_setup/rp2-pico-latest.uf2" ]; then
            if cp "$HOME/rp2040_setup/rp2-pico-latest.uf2" "$RPI_MOUNT/"; then
                echo "✓ MicroPython firmware flashed!"
                echo "RP2040 will now restart with MicroPython..."
                sleep 3
            else
                echo "✗ Failed to copy firmware file"
                echo "Check permissions or try:"
                echo "  sudo cp $HOME/rp2040_setup/rp2-pico-latest.uf2 $RPI_MOUNT/"
                return 1
            fi
        else
            echo "✗ MicroPython firmware not found"
            echo "Please download it first with:"
            echo "  wget https://micropython.org/download/rp2-pico/rp2-pico-latest.uf2 -O ~/rp2040_setup/rp2-pico-latest.uf2"
            return 1
        fi
    else
        echo
        echo "Troubleshooting steps:"
        echo "1. Make sure you're using a data USB cable (not power-only)"
        echo "2. Try a different USB port"
        echo "3. Hold BOOTSEL button firmly while connecting"
        echo "4. Check if device appears:"
        echo "   lsusb | grep -i pico"
        echo "   lsblk | grep -i rpi"
        echo
        return 1
    fi
}

# Upload firmware code
upload_firmware() {
    echo
    echo "Step 2: Upload firmware code"
    
    # Wait for device to reappear
    echo "Waiting for RP2040 to restart with MicroPython..."
    sleep 5
    
    if check_rp2040; then
        PORT=$(ls /dev/ttyACM* 2>/dev/null | head -1)
        if [ -z "$PORT" ]; then
            PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
        fi
        
        echo "Uploading firmware to $PORT..."
        
        # Check if we have mpremote
        if command -v mpremote >/dev/null 2>&1; then
            echo "Using mpremote to upload firmware..."
            mpremote connect "$PORT" cp "$(pwd)/src/new_bot/scripts/rp2040_firmware/main.py" :main.py
            echo "✓ Primary firmware uploaded!"
        elif command -v thonny >/dev/null 2>&1; then
            echo "Please use Thonny to upload the firmware:"
            echo "1. Open Thonny"
            echo "2. Go to Tools > Options > Interpreter"
            echo "3. Select 'MicroPython (Raspberry Pi Pico)'"
            echo "4. Choose port: $PORT"
            echo "5. Open: src/new_bot/scripts/rp2040_firmware/main.py"
            echo "6. Save as main.py on the RP2040"
            read -p "Press Enter when done..."
        else
            echo "✗ No upload tool found. Please install mpremote or use Thonny"
            exit 1
        fi
    else
        echo "✗ Cannot find RP2040 device after flashing"
        exit 1
    fi
}

# Test firmware
test_firmware() {
    echo
    echo "Step 3: Test firmware communication"
    
    if check_rp2040; then
        PORT=$(ls /dev/ttyACM* 2>/dev/null | head -1)
        if [ -z "$PORT" ]; then
            PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
        fi
        
        echo "Testing communication with $PORT..."
        python3 "$(pwd)/src/new_bot/scripts/test_rp2040.py" --port "$PORT"
    else
        echo "✗ Cannot find RP2040 device for testing"
        exit 1
    fi
}

# Main menu
main_menu() {
    echo
    echo "Choose an option:"
    echo "1. Flash MicroPython firmware (first time setup)"
    echo "2. Upload firmware code only"
    echo "3. Test firmware communication"
    echo "4. Run complete setup (1+2+3)"
    echo "5. Exit"
    echo
    
    read -p "Enter choice (1-5): " choice
    
    case $choice in
        1)
            flash_micropython
            ;;
        2)
            upload_firmware
            ;;
        3)
            test_firmware
            ;;
        4)
            flash_micropython
            upload_firmware
            test_firmware
            ;;
        5)
            echo "Goodbye!"
            exit 0
            ;;
        *)
            echo "Invalid choice"
            main_menu
            ;;
    esac
}

# Check prerequisites
echo "Checking prerequisites..."

if ! command -v python3 >/dev/null 2>&1; then
    echo "✗ Python3 not found"
    exit 1
fi

if ! python3 -c "import serial" 2>/dev/null; then
    echo "Installing pyserial..."
    sudo apt install -y python3-serial
fi

echo "✓ Prerequisites checked"

# Run main menu
main_menu
