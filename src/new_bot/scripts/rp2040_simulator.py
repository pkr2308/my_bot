#!/usr/bin/env python3

"""
RP2040 Simulator
This script simulates an RP2040 microcontroller for testing the ROS2 hardware interface
without requiring physical hardware.
"""

import serial
import time
import math
import threading
import sys
import signal

class RP2040Simulator:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
        # Simulated sensor data
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # 4 joints
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0]
        
        # IMU simulation
        self.sim_time = 0.0
        self.imu_quat = [0.0, 0.0, 0.0, 1.0]  # x,y,z,w quaternion
        self.imu_gyro = [0.0, 0.0, 0.0]       # angular velocity
        self.imu_accel = [0.0, 0.0, 9.81]     # linear acceleration
        
        # Range sensors simulation  
        self.ranges = [2.0, 1.5, 1.8]  # front, left, right in meters
        
        # Control commands
        self.target_positions = [0.0, 0.0, 0.0, 0.0]
        self.target_velocities = [0.0, 0.0, 0.0, 0.0]
        
        print(f"RP2040 Simulator initialized for {port} at {baudrate} baud")
    
    def connect(self):
        """Connect to virtual serial port"""
        try:
            # In a real scenario, you might use socat to create virtual serial ports
            # For now, we'll create a simple simulation
            print("RP2040 Simulator: Virtual connection established")
            self.running = True
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from virtual serial port"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        print("RP2040 Simulator: Disconnected")
    
    def update_simulation(self):
        """Update simulated sensor values"""
        self.sim_time += 0.01  # 10ms increment
        
        # Simulate joint dynamics (simple first-order response)
        for i in range(len(self.joint_positions)):
            # Position control simulation
            pos_error = self.target_positions[i] - self.joint_positions[i]
            self.joint_positions[i] += pos_error * 0.1  # Simple P controller
            
            # Velocity control simulation  
            vel_error = self.target_velocities[i] - self.joint_velocities[i]
            self.joint_velocities[i] += vel_error * 0.2
        
        # Simulate IMU with some noise and motion
        self.imu_gyro[0] = 0.1 * math.sin(self.sim_time * 0.5) + self.get_noise(0.01)
        self.imu_gyro[1] = 0.05 * math.cos(self.sim_time * 0.3) + self.get_noise(0.01)
        self.imu_gyro[2] = 0.02 * math.sin(self.sim_time * 0.7) + self.get_noise(0.005)
        
        # Integrate gyro to get orientation (simplified)
        dt = 0.01
        self.imu_quat[0] += self.imu_gyro[0] * dt * 0.5
        self.imu_quat[1] += self.imu_gyro[1] * dt * 0.5 
        self.imu_quat[2] += self.imu_gyro[2] * dt * 0.5
        
        # Normalize quaternion
        quat_norm = math.sqrt(sum(q*q for q in self.imu_quat))
        if quat_norm > 0:
            self.imu_quat = [q/quat_norm for q in self.imu_quat]
        
        # Simulate accelerometer with gravity and some motion
        self.imu_accel[0] = math.sin(self.sim_time * 0.2) * 0.5 + self.get_noise(0.1)
        self.imu_accel[1] = math.cos(self.sim_time * 0.3) * 0.3 + self.get_noise(0.1)
        self.imu_accel[2] = 9.81 + math.sin(self.sim_time * 0.1) * 0.2 + self.get_noise(0.1)
        
        # Simulate ranging sensors with some variation
        self.ranges[0] = 2.0 + 0.5 * math.sin(self.sim_time * 0.1) + self.get_noise(0.05)  # front
        self.ranges[1] = 1.5 + 0.3 * math.cos(self.sim_time * 0.15) + self.get_noise(0.05)  # left  
        self.ranges[2] = 1.8 + 0.4 * math.sin(self.sim_time * 0.12) + self.get_noise(0.05)  # right
        
        # Clamp ranges to reasonable values
        self.ranges = [max(0.1, min(10.0, r)) for r in self.ranges]
    
    def get_noise(self, magnitude):
        """Generate Gaussian noise"""
        import random
        return random.gauss(0, magnitude)
    
    def format_sensor_data(self):
        """Format sensor data as expected by ROS interface"""
        data_parts = []
        
        # Joint positions and velocities
        for i in range(len(self.joint_positions)):
            data_parts.append(f"{self.joint_positions[i]:.3f}")
            data_parts.append(f"{self.joint_velocities[i]:.3f}")
        
        # IMU quaternion
        for q in self.imu_quat:
            data_parts.append(f"{q:.6f}")
        
        # IMU gyroscope
        for g in self.imu_gyro:
            data_parts.append(f"{g:.6f}")
        
        # IMU accelerometer
        for a in self.imu_accel:
            data_parts.append(f"{a:.6f}")
        
        # Range sensors
        for r in self.ranges:
            data_parts.append(f"{r:.3f}")
        
        return "<" + ",".join(data_parts) + ">"
    
    def parse_command(self, command):
        """Parse incoming commands"""
        if command == "PING":
            return "<PONG>"
        elif command == "INIT":
            print("RP2040 Simulator: Initialized")
            return "<OK>"
        elif command == "STOP":
            print("RP2040 Simulator: Stopped")
            # Reset all targets to zero
            self.target_positions = [0.0] * 4
            self.target_velocities = [0.0] * 4
            return "<OK>"
        elif command == "READ":
            return self.format_sensor_data()
        elif command.startswith("CMD"):
            # Parse control command: CMD,P:val,P:val,V:val,V:val
            try:
                parts = command.split(',')[1:]  # Skip "CMD"
                joint_idx = 0
                
                for part in parts:
                    if ':' in part:
                        cmd_type, value = part.split(':')
                        val = float(value)
                        
                        if cmd_type == 'P' and joint_idx < len(self.target_positions):
                            self.target_positions[joint_idx] = val
                        elif cmd_type == 'V' and joint_idx < len(self.target_velocities):
                            self.target_velocities[joint_idx] = val
                        
                        joint_idx += 1
                
                return "<OK>"
            except Exception as e:
                print(f"RP2040 Simulator: Failed to parse command: {e}")
                return "<ERROR>"
        else:
            print(f"RP2040 Simulator: Unknown command: {command}")
            return "<UNKNOWN>"
    
    def run_simulation(self):
        """Run the main simulation loop"""
        print("RP2040 Simulator: Starting simulation...")
        print("Press Ctrl+C to stop")
        
        if not self.connect():
            return False
        
        try:
            while self.running:
                self.update_simulation()
                time.sleep(0.01)  # 100Hz simulation
                
        except KeyboardInterrupt:
            print("\nRP2040 Simulator: Shutting down...")
        finally:
            self.disconnect()
        
        return True
    
    def run_serial_simulation(self, port='/tmp/vserial'):
        """Run simulation with actual serial communication"""
        try:
            # This would require socat to create virtual serial ports
            # socat -d -d pty,raw,echo=0 pty,raw,echo=0
            print("Serial simulation not implemented yet")
            print("Use the test script in interactive mode instead")
            return False
        except Exception as e:
            print(f"Serial simulation failed: {e}")
            return False


class InteractiveSimulator:
    """Interactive mode for testing commands"""
    
    def __init__(self):
        self.sim = RP2040Simulator()
    
    def run(self):
        print("\n=== RP2040 Interactive Simulator ===")
        print("Available commands:")
        print("  PING       - Test basic communication")  
        print("  INIT       - Initialize the system")
        print("  READ       - Get sensor data")
        print("  CMD,P:0.5,P:-0.3,V:1.0,V:0.0  - Send control commands")
        print("  STOP       - Stop all motion")
        print("  quit       - Exit simulator")
        print()
        
        # Start background simulation
        sim_thread = threading.Thread(target=self.sim.run_simulation, daemon=True)
        sim_thread.start()
        
        while True:
            try:
                command = input("RP2040> ").strip()
                
                if command.lower() in ['quit', 'exit', 'q']:
                    break
                
                if command:
                    response = self.sim.parse_command(command)
                    print(f"Response: {response}")
                    
            except KeyboardInterrupt:
                break
        
        print("\nGoodbye!")
        self.sim.disconnect()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='RP2040 Simulator')
    parser.add_argument('--interactive', '-i', action='store_true', 
                       help='Run in interactive mode')
    parser.add_argument('--port', default='/dev/ttyACM0', 
                       help='Serial port (for future use)')
    
    args = parser.parse_args()
    
    if args.interactive:
        sim = InteractiveSimulator()
        sim.run()
    else:
        sim = RP2040Simulator(args.port)
        sim.run_simulation()


if __name__ == '__main__':
    main()
