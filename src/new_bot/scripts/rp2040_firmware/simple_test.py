"""
Simple RP2040 Firmware for Initial Testing
This is a minimal version to test basic communication
"""

import time
import sys

# Simple LED blink on pin 25 (built-in LED)
try:
    from machine import Pin
    led = Pin(25, Pin.OUT)
    has_hardware = True
except ImportError:
    has_hardware = False
    print("Running in simulation mode")

class SimpleRP2040:
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
        
        print("Simple RP2040 Controller ready")
    
    def process_command(self, cmd):
        """Process simple commands"""
        cmd = cmd.strip().upper()
        
        if cmd == "PING":
            return "PONG"
        elif cmd == "INIT":
            self.initialized = True
            return "OK"
        elif cmd == "STOP":
            self.initialized = False
            return "OK"
        elif cmd == "READ":
            # Return sensor data
            data = []
            data.extend([f"{p:.3f}" for p in self.joint_pos])
            data.extend([f"{v:.3f}" for v in self.joint_vel])
            data.extend([f"{q:.6f}" for q in self.imu_quat])
            data.extend([f"{g:.6f}" for g in self.imu_gyro])
            data.extend([f"{a:.6f}" for a in self.imu_accel])
            data.extend([f"{r:.3f}" for r in self.ranges])
            return "<" + ",".join(data) + ">"
        else:
            return "UNKNOWN"
    
    def update(self):
        """Update simulated data"""
        self.counter += 1
        
        # Simulate some changing values
        t = self.counter * 0.01
        import math
        
        self.joint_pos[0] = 0.1 * math.sin(t * 0.5)
        self.joint_vel[0] = 0.05 * math.cos(t * 0.5)
        
        self.imu_gyro[2] = 0.02 * math.sin(t * 0.3)
        self.ranges[0] = 2.0 + 0.2 * math.sin(t * 0.2)
        
        # Blink LED if available
        if has_hardware and self.counter % 100 == 0:
            led.toggle()
    
    def run_interactive(self):
        """Run in interactive mode for testing"""
        print("\nSimple RP2040 Interactive Mode")
        print("Commands: PING, INIT, READ, STOP, quit")
        
        while True:
            try:
                cmd = input("RP2040> ").strip()
                if cmd.lower() in ['quit', 'exit']:
                    break
                
                if cmd:
                    response = self.process_command(cmd)
                    print(f"< {response}")
                    
                self.update()
                    
            except KeyboardInterrupt:
                break
        
        print("Goodbye!")

# If running on PC (for testing)
if __name__ == "__main__" and not has_hardware:
    controller = SimpleRP2040()
    controller.run_interactive()

# If running on RP2040
elif __name__ == "__main__" and has_hardware:
    controller = SimpleRP2040()
    
    while True:
        try:
            # In real RP2040, you'd read from UART here
            # For now, just update and blink
            controller.update()
            time.sleep_ms(10)
            
        except KeyboardInterrupt:
            break
