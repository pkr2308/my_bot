#!/usr/bin/env python3

"""
RP2040 Communication Test Script
This script helps test the serial communication with your RP2040 microcontroller
"""

import serial
import time
import json

class RP2040Tester:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        
    def connect(self):
        """Connect to RP2040"""
        try:
            self.serial_conn = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=1.0
            )
            time.sleep(2)  # Wait for connection to stabilize
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            print(f"✓ Connected to RP2040 on {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from RP2040"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("✓ Disconnected from RP2040")
    
    def send_command(self, command):
        """Send command to RP2040"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("✗ Not connected to RP2040")
            return False
            
        try:
            full_command = f"<{command}>\n"
            self.serial_conn.write(full_command.encode())
            self.serial_conn.flush()
            print(f"→ Sent: {full_command.strip()}")
            return True
        except Exception as e:
            print(f"✗ Failed to send command: {e}")
            return False
    
    def read_response(self):
        """Read response from RP2040"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
            
        try:
            # Wait a bit for the response
            time.sleep(0.1)
            
            # Check if data is available
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode().strip()
                if response:
                    print(f"← Received: {response}")
                return response
            else:
                # No immediate response, wait a bit more
                time.sleep(0.2)
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.readline().decode().strip()
                    if response:
                        print(f"← Received: {response}")
                    return response
            return None
        except Exception as e:
            print(f"✗ Failed to read response: {e}")
            return None
    
    def test_basic_communication(self):
        """Test basic communication"""
        print("\n=== Testing Basic Communication ===")
        
        # Test ping
        self.send_command("PING")
        response = self.read_response()
        
        if response and "PONG" in response:
            print("✓ Basic communication working")
            return True
        else:
            print("✗ Basic communication failed")
            return False
    
    def test_sensor_reading(self):
        """Test sensor data reading"""
        print("\n=== Testing Sensor Reading ===")
        
        self.send_command("READ")
        response = self.read_response()
        
        if response and response.startswith('<') and response.endswith('>'):
            # Parse sensor data
            data = response[1:-1]  # Remove < >
            values = data.split(',')
            
            print(f"✓ Received sensor data with {len(values)} values:")
            
            try:
                idx = 0
                print("  Joint Positions & Velocities:")
                for i in range(4):  # 4 joints
                    pos = float(values[idx])
                    vel = float(values[idx + 1])
                    print(f"    Joint {i}: pos={pos:.3f}, vel={vel:.3f}")
                    idx += 2
                
                print("  IMU Data:")
                print(f"    Orientation: qx={float(values[idx]):.3f}, qy={float(values[idx+1]):.3f}, qz={float(values[idx+2]):.3f}, qw={float(values[idx+3]):.3f}")
                idx += 4
                print(f"    Angular Vel: gx={float(values[idx]):.3f}, gy={float(values[idx+1]):.3f}, gz={float(values[idx+2]):.3f}")
                idx += 3
                print(f"    Linear Accel: ax={float(values[idx]):.3f}, ay={float(values[idx+1]):.3f}, az={float(values[idx+2]):.3f}")
                idx += 3
                
                print("  Range Sensors:")
                print(f"    Front: {float(values[idx]):.3f}m, Left: {float(values[idx+1]):.3f}m, Right: {float(values[idx+2]):.3f}m")
                
                return True
                
            except (ValueError, IndexError) as e:
                print(f"✗ Failed to parse sensor data: {e}")
                return False
        else:
            print("✗ No valid sensor data received")
            return False
    
    def test_actuator_control(self):
        """Test actuator control"""
        print("\n=== Testing Actuator Control ===")
        
        # Test steering servo
        print("Testing steering servo...")
        self.send_command("CMD,P:0.5,P:0.5,V:0.0,V:0.0")  # Steer right
        time.sleep(1)
        
        self.send_command("CMD,P:-0.5,P:-0.5,V:0.0,V:0.0")  # Steer left
        time.sleep(1)
        
        self.send_command("CMD,P:0.0,P:0.0,V:0.0,V:0.0")  # Center steering
        time.sleep(1)
        
        # Test drive motor
        print("Testing drive motor...")
        self.send_command("CMD,P:0.0,P:0.0,V:1.0,V:1.0")  # Forward
        time.sleep(2)
        
        self.send_command("CMD,P:0.0,P:0.0,V:-1.0,V:-1.0")  # Backward
        time.sleep(2)
        
        self.send_command("CMD,P:0.0,P:0.0,V:0.0,V:0.0")  # Stop
        
        print("✓ Actuator test completed")
        return True
    
    def run_full_test(self):
        """Run complete test suite"""
        print("=== RP2040 Hardware Interface Test ===")
        
        if not self.connect():
            return False
        
        try:
            # Run tests
            tests_passed = 0
            total_tests = 3
            
            if self.test_basic_communication():
                tests_passed += 1
            
            if self.test_sensor_reading():
                tests_passed += 1
            
            if self.test_actuator_control():
                tests_passed += 1
            
            print(f"\n=== Test Results ===")
            print(f"Passed: {tests_passed}/{total_tests}")
            
            if tests_passed == total_tests:
                print("✓ All tests passed! RP2040 interface is ready.")
                return True
            else:
                print("✗ Some tests failed. Check RP2040 firmware and connections.")
                return False
                
        finally:
            self.disconnect()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Test RP2040 communication')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    
    args = parser.parse_args()
    
    tester = RP2040Tester(args.port, args.baud)
    success = tester.run_full_test()
    
    exit(0 if success else 1)


if __name__ == '__main__':
    main()
