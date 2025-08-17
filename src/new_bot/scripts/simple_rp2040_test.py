#!/usr/bin/env python3

import serial
import time
import sys

def test_rp2040_simple(port='/dev/ttyACM0'):
    """Simple RP2040 communication test"""
    try:
        print(f"Connecting to {port}...")
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(2)  # Wait for connection to stabilize
        
        print("Testing PING command...")
        
        # Clear any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Send PING command
        command = "PING\n"
        print(f"→ Sending: {command.strip()}")
        ser.write(command.encode())
        
        # Read response
        time.sleep(0.5)
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"← Received: {response}")
            
            if response == "PONG":
                print("✓ Communication successful!")
                return True
            else:
                print(f"✗ Unexpected response: {response}")
        else:
            print("✗ No response received")
        
        ser.close()
        return False
        
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    test_rp2040_simple(port)
