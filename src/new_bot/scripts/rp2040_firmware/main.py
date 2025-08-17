"""
RP2040 Firmware for ROS2 - Non-blocking version
"""

import sys
import time
import math
import select

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
        
        # Handle empty commands
        if not cmd:
            return None
        
        # Remove < and > markers if present
        if cmd.startswith('<') and cmd.endswith('>'):
            cmd = cmd[1:-1]
        
        # Handle empty commands after bracket removal
        if not cmd:
            return None
        
        cmd = cmd.upper()
        
        if cmd == "PING":
            return "PONG"
        elif cmd == "INIT":
            self.initialized = True
            if has_hardware:
                led.on()
            return "OK"
        elif cmd == "STOP":
            self.initialized = False
            if has_hardware:
                led.off()
            return "OK"
        elif cmd == "READ":
            return self.get_sensor_data()
        elif cmd.startswith("CMD"):
            return self.process_control_cmd(cmd)
        else:
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
        except Exception:
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

# Main loop - this runs when the file is executed
def handle_input():
    try:
        line = sys.stdin.readline()
        if line:
            line = line.strip()
            if line:  # Only process non-empty lines
                response = controller.process_command(line)
                if response:  # Only print non-None responses
                    print(response)
                    sys.stdout.flush()  # Ensure immediate output
    except Exception as e:
        # Don't print errors to avoid confusing the host
        pass

# Check if this is being run as main
if __name__ == "__main__":
    print("RP2040 Controller ready")
    while True:
        try:
            handle_input()
        except KeyboardInterrupt:
            break
else:
    # If imported, set up for REPL use
    print("RP2040 Controller loaded - use handle_input() for commands")
