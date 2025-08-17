"""
RP2040 Debug Firmware - Shows what commands are received
"""

import sys
import time
import math

try:
    from machine import Pin
    led = Pin(25, Pin.OUT)
    has_hardware = True
except ImportError:
    has_hardware = False

class RP2040Controller:
    def __init__(self):
        self.counter = 0
        self.initialized = False
        
        # Simulated sensor data
        self.joint_pos = [0.0, 0.0, 0.0, 0.0]
        self.joint_vel = [0.0, 0.0, 0.0, 0.0]
        self.imu_quat = [0.0, 0.0, 0.0, 1.0]
        self.imu_gyro = [0.0, 0.0, 0.0]
        self.imu_accel = [0.0, 0.0, 9.81]
        self.ranges = [2.0, 1.5, 1.8]
        
    def process_command(self, cmd):
        """Process commands from ROS2"""
        original_cmd = cmd
        cmd = cmd.strip()
        
        # Debug: Print what we received
        print(f"DEBUG: Received raw: '{original_cmd}' (len={len(original_cmd)})")
        print(f"DEBUG: After strip: '{cmd}' (len={len(cmd)})")
        
        # Remove < and > markers if present
        if cmd.startswith('<') and cmd.endswith('>'):
            cmd = cmd[1:-1]
            print(f"DEBUG: After removing brackets: '{cmd}'")
        
        cmd = cmd.upper()
        print(f"DEBUG: After upper: '{cmd}'")
        
        if cmd == "PING":
            print("DEBUG: Matched PING command")
            return "PONG"
        elif cmd == "INIT":
            print("DEBUG: Matched INIT command")
            self.initialized = True
            if has_hardware:
                led.on()
            return "OK"
        elif cmd == "STOP":
            print("DEBUG: Matched STOP command")
            self.initialized = False
            if has_hardware:
                led.off()
            return "OK"
        elif cmd == "READ":
            print("DEBUG: Matched READ command")
            return self.get_sensor_data()
        elif cmd.startswith("CMD"):
            print("DEBUG: Matched CMD command")
            return self.process_control_cmd(cmd)
        else:
            print(f"DEBUG: No match found for '{cmd}'")
            return "UNKNOWN"
    
    def process_control_cmd(self, cmd):
        """Process control command"""
        try:
            parts = cmd.split(',')[1:]  # Skip "CMD"
            joint_idx = 0
            
            for part in parts:
                if ':' in part and joint_idx < 4:
                    cmd_type, value = part.split(':')
                    val = float(value)
                    
                    if cmd_type == 'P':
                        self.joint_pos[joint_idx] = val * 0.1
                    elif cmd_type == 'V':
                        self.joint_vel[joint_idx] = val * 0.1
                    
                    joint_idx += 1
            
            return "OK"
        except Exception as e:
            print(f"DEBUG: CMD error: {e}")
            return "ERROR"
    
    def get_sensor_data(self):
        """Return formatted sensor data"""
        self.update_simulation()
        
        data = []
        # Joint positions and velocities
        for i in range(4):
            data.append(f"{self.joint_pos[i]:.3f}")
            data.append(f"{self.joint_vel[i]:.3f}")
        
        # IMU quaternion
        for q in self.imu_quat:
            data.append(f"{q:.6f}")
        
        # IMU gyroscope  
        for g in self.imu_gyro:
            data.append(f"{g:.6f}")
        
        # IMU accelerometer
        for a in self.imu_accel:
            data.append(f"{a:.6f}")
        
        # Range sensors
        for r in self.ranges:
            data.append(f"{r:.3f}")
        
        return "<" + ",".join(data) + ">"
    
    def update_simulation(self):
        """Update simulated sensor values"""
        self.counter += 1
        t = self.counter * 0.01
        
        self.joint_pos[0] = 0.1 * math.sin(t * 0.5)
        self.joint_vel[0] = 0.05 * math.cos(t * 0.5)
        self.imu_gyro[2] = 0.02 * math.sin(t * 0.3)
        self.ranges[0] = 2.0 + 0.2 * math.sin(t * 0.2)
        
        if has_hardware and self.counter % 100 == 0:
            led.toggle()

# Create controller instance
controller = RP2040Controller()

# Main loop
def handle_input():
    try:
        line = sys.stdin.readline().strip()
        if line:
            response = controller.process_command(line)
            print(response)
            sys.stdout.flush()  # Ensure immediate output
    except Exception as e:
        print(f"ERROR: {e}")

# Check if this is being run as main
if __name__ == "__main__":
    print("RP2040 Debug Controller ready")
    while True:
        try:
            handle_input()
        except KeyboardInterrupt:
            break
else:
    # If imported, set up for REPL use
    print("RP2040 Debug Controller loaded")
