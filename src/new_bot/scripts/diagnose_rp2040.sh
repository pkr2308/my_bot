#!/bin/bash

# RP2040 Connection Diagnostic Tool
echo "=== RP2040 Connection Diagnostic ==="
echo

echo "1. Checking USB devices..."
echo "All USB devices:"
lsusb
echo

if lsusb | grep -E "(2e8a|Raspberry|Pico)"; then
    echo "✓ RP2040/Pico device found in USB!"
else
    echo "✗ No RP2040/Pico device found in USB"
fi
echo

echo "2. Checking block devices..."
echo "All block devices:"
lsblk
echo

if lsblk | grep -i rpi; then
    echo "✓ RPI device found in block devices!"
else
    echo "✗ No RPI device found in block devices"
fi
echo

echo "3. Checking serial devices..."
if ls /dev/ttyACM* >/dev/null 2>&1; then
    echo "✓ Found serial devices: $(ls /dev/ttyACM*)"
elif ls /dev/ttyUSB* >/dev/null 2>&1; then
    echo "✓ Found serial devices: $(ls /dev/ttyUSB*)"
else
    echo "✗ No serial devices found"
fi
echo

echo "4. Checking mounted drives..."
echo "Currently mounted drives:"
mount | grep -E "(media|mnt|RPI)" || echo "  No relevant mounts found"
echo

echo "5. Checking for RPI-RP2 specifically..."
if [ -d "/media/$USER/RPI-RP2" ]; then
    echo "✓ Found RPI-RP2 at /media/$USER/RPI-RP2"
    ls -la "/media/$USER/RPI-RP2"
elif [ -d "/media/RPI-RP2" ]; then
    echo "✓ Found RPI-RP2 at /media/RPI-RP2" 
    ls -la "/media/RPI-RP2"
elif [ -d "/mnt/RPI-RP2" ]; then
    echo "✓ Found RPI-RP2 at /mnt/RPI-RP2"
    ls -la "/mnt/RPI-RP2"
else
    echo "✗ RPI-RP2 drive not found in common locations"
fi
echo

echo "=== Diagnostic Complete ==="
echo
echo "If your RP2040 is NOT detected:"
echo "1. Make sure it's physically connected via USB"
echo "2. Try a different USB cable (must be data cable, not power-only)"
echo "3. Try a different USB port"
echo "4. For bootloader mode: Hold BOOTSEL while connecting USB"
echo
echo "If your RP2040 IS detected but RPI-RP2 drive not found:"
echo "1. It might be in normal MicroPython mode (not bootloader)"
echo "2. To enter bootloader: Hold BOOTSEL + press RESET, or disconnect/reconnect while holding BOOTSEL"
echo
echo "Expected USB IDs:"
echo "  - Bootloader mode: 2e8a:0003 (RP2 Boot)"  
echo "  - MicroPython mode: 2e8a:0005 (Board in FS mode)"
