#!/usr/bin/env python3

import serial
import time
import sys

def test_ping_debug(port='/dev/ttyACM0'):
    """Debug PING command to see what's happening"""
    try:
        print(f"Connecting to {port}...")
        ser = serial.Serial(port, 115200, timeout=3)
        time.sleep(3)  # Wait for connection to stabilize
        
        # Clear any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print("\nTesting different PING command formats:")
        
        test_commands = [
            "PING\n",
            "<PING>\n", 
            "ping\n",
            "<ping>\n",
            "PING\r\n",
            "<PING>\r\n"
        ]
        
        for i, command in enumerate(test_commands):
            print(f"\n--- Test {i+1}: {repr(command)} ---")
            
            # Clear buffers
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Send command
            print(f"→ Sending: {repr(command)}")
            ser.write(command.encode())
            
            # Wait and read all responses
            time.sleep(1)
            responses = []
            
            while ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        responses.append(line)
                        print(f"← Received: {repr(line)}")
                except:
                    break
            
            if not responses:
                print("← No response received")
        
        print("\n" + "="*50)
        print("Debug session complete")
        ser.close()
        
    except Exception as e:
        print(f"✗ Error: {e}")

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    test_ping_debug(port)
