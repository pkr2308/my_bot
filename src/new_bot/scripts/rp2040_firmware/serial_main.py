"""
RP2040 Firmware for ROS2 Hardware Interface
Serial Communication Version
"""

import sys
import time
import math

# Try to import hardware-specific modules
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
        
        print("RP2040 Controller initialized")
    
    def process_command(self, cmd):
        """Process commands from ROS2"""
        cmd = cmd.strip().upper()
        
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
            # Process control command
            return self.process_control_cmd(cmd)
        else:
            return "UNKNOWN"
    
    def process_control_cmd(self, cmd):
        """Process control command: CMD,P:val,P:val,V:val,V:val"""
        try:
            parts = cmd.split(',')[1:]  # Skip "CMD"
            joint_idx = 0
            
            for part in parts:
                if ':' in part and joint_idx < 4:
                    cmd_type, value = part.split(':')
                    val = float(value)
                    
                    if cmd_type == 'P':
                        # Set position target (simplified)
                        self.joint_pos[joint_idx] = val * 0.1  # Scale down for simulation
                    elif cmd_type == 'V':
                        # Set velocity target (simplified)
                        self.joint_vel[joint_idx] = val * 0.1  # Scale down for simulation
                    
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
        
        # Simulate some changing values
        self.joint_pos[0] = 0.1 * math.sin(t * 0.5)
        self.joint_vel[0] = 0.05 * math.cos(t * 0.5)
        
        self.imu_gyro[2] = 0.02 * math.sin(t * 0.3)
        self.ranges[0] = 2.0 + 0.2 * math.sin(t * 0.2)
        
        # Blink LED if available
        if has_hardware and self.counter % 100 == 0:
            led.toggle()

# Main execution
def main():
    controller = RP2040Controller()
    
    print("RP2040 ready for serial communication")
    print("Waiting for commands...")
    
    while True:
        try:
            # Read line from stdin (USB serial)
            line = input().strip()
            
            if line:
                # Process command and send response
                response = controller.process_command(line)
                print(response)
                
        except KeyboardInterrupt:
            print("Shutting down...")
            break
        except EOFError:
            # Handle disconnection
            time.sleep(0.1)
            continue
        except Exception as e:
            print("ERROR")

if __name__ == "__main__":
    main()
