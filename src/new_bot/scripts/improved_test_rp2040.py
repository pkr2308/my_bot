#!/usr/bin/env python3

"""
Improved RP2040 Communication Test Script with better error handling
"""

import serial
import time
import sys

class RP2040Tester:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.connection_retries = 3
        self.command_timeout = 2.0
    
    def connect(self):
        """Connect to RP2040 with retry logic"""
        for attempt in range(self.connection_retries):
            try:
                print(f"Connecting to RP2040 on {self.port} (attempt {attempt + 1})...")
                self.serial_conn = serial.Serial(
                    self.port, 
                    self.baudrate, 
                    timeout=self.command_timeout,
                    write_timeout=1.0
                )
                
                # Wait for connection to stabilize
                time.sleep(2)
                
                # Clear any leftover data
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                
                print(f"✓ Connected to RP2040 on {self.port} at {self.baudrate} baud")
                return True
                
            except serial.SerialException as e:
                print(f"✗ Connection attempt {attempt + 1} failed: {e}")
                if attempt < self.connection_retries - 1:
                    time.sleep(1)
                    
        return False
    
    def send_command_and_wait(self, command, expected_response=None, max_wait=3.0):
        """Send command and wait for response with better timing"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("✗ Not connected to RP2040")
            return None
            
        try:
            # Clear buffers first
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            # Send command
            full_command = f"<{command}>\n"
            print(f"→ Sent: {full_command.strip()}")
            self.serial_conn.write(full_command.encode())
            self.serial_conn.flush()
            
            # Wait for response with multiple attempts
            start_time = time.time()
            response = None
            
            while time.time() - start_time < max_wait:
                if self.serial_conn.in_waiting > 0:
                    try:
                        response = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            print(f"← Received: {response}")
                            break
                    except:
                        pass
                time.sleep(0.05)  # Small delay between checks
            
            if not response:
                print(f"✗ No response within {max_wait}s")
                
            return response
            
        except Exception as e:
            print(f"✗ Communication error: {e}")
            return None
    
    def test_ping(self):
        """Test basic PING communication"""
        print("\n=== Testing PING Command ===")
        response = self.send_command_and_wait("PING", expected_response="PONG")
        
        if response == "PONG":
            print("✓ PING test successful")
            return True
        else:
            print(f"✗ PING test failed - expected 'PONG', got '{response}'")
            return False
    
    def test_sensor_reading(self):
        """Test sensor data reading"""
        print("\n=== Testing Sensor Reading ===")
        response = self.send_command_and_wait("READ", max_wait=3.0)
        
        if response and response.startswith('<') and response.endswith('>'):
            data = response[1:-1]  # Remove < >
            values = data.split(',')
            
            print(f"✓ Received sensor data with {len(values)} values")
            
            # Parse and display the data
            try:
                idx = 0
                print("  Joint Data:")
                for i in range(4):
                    pos = float(values[idx])
                    vel = float(values[idx + 1])
                    print(f"    Joint {i}: pos={pos:.3f}, vel={vel:.3f}")
                    idx += 2
                
                print("  IMU Data:")
                quat = [float(values[idx + i]) for i in range(4)]
                print(f"    Quaternion: x={quat[0]:.3f}, y={quat[1]:.3f}, z={quat[2]:.3f}, w={quat[3]:.3f}")
                idx += 4
                
                gyro = [float(values[idx + i]) for i in range(3)]
                print(f"    Gyroscope: x={gyro[0]:.3f}, y={gyro[1]:.3f}, z={gyro[2]:.3f}")
                idx += 3
                
                accel = [float(values[idx + i]) for i in range(3)]
                print(f"    Accelerometer: x={accel[0]:.3f}, y={accel[1]:.3f}, z={accel[2]:.3f}")
                idx += 3
                
                ranges = [float(values[idx + i]) for i in range(3)]
                print(f"    Range Sensors: front={ranges[0]:.3f}m, left={ranges[1]:.3f}m, right={ranges[2]:.3f}m")
                
                return True
            except Exception as e:
                print(f"✗ Error parsing sensor data: {e}")
                return False
        else:
            print(f"✗ Invalid sensor response: {response}")
            return False
    
    def test_actuator_commands(self):
        """Test actuator control commands"""
        print("\n=== Testing Actuator Commands ===")
        
        test_commands = [
            "CMD,P:0.5,P:0.5,V:0.0,V:0.0",  # Position command
            "CMD,P:0.0,P:0.0,V:1.0,V:1.0",  # Velocity command
            "CMD,P:0.0,P:0.0,V:0.0,V:0.0"   # Stop command
        ]
        
        success_count = 0
        for i, cmd in enumerate(test_commands):
            print(f"Testing actuator command {i+1}...")
            response = self.send_command_and_wait(cmd, max_wait=1.0)
            
            if response == "OK":
                success_count += 1
                print(f"✓ Command {i+1} successful")
            else:
                print(f"✗ Command {i+1} failed - got '{response}'")
        
        if success_count == len(test_commands):
            print("✓ All actuator commands successful")
            return True
        else:
            print(f"✗ {len(test_commands) - success_count} actuator commands failed")
            return False
    
    def disconnect(self):
        """Disconnect from RP2040"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("✓ Disconnected from RP2040")
    
    def run_full_test(self):
        """Run complete test suite"""
        print("=== RP2040 Hardware Interface Test (Improved) ===")
        
        if not self.connect():
            print("✗ Connection failed - aborting tests")
            return False
        
        test_results = []
        
        try:
            # Test basic communication
            test_results.append(self.test_ping())
            
            # Test sensor reading
            test_results.append(self.test_sensor_reading())
            
            # Test actuator commands
            test_results.append(self.test_actuator_commands())
            
        finally:
            self.disconnect()
        
        # Summary
        passed = sum(test_results)
        total = len(test_results)
        
        print(f"\n=== Test Results ===")
        print(f"Passed: {passed}/{total}")
        
        if passed == total:
            print("✓ All tests passed! RP2040 communication is working correctly.")
            return True
        else:
            print("✗ Some tests failed. Check RP2040 firmware and connections.")
            return False

def main():
    """Main function"""
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    tester = RP2040Tester(port)
    success = tester.run_full_test()
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
