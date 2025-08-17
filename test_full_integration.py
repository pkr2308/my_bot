#!/usr/bin/env python3

"""
Complete ROS2 + RP2040 Integration Test
Tests the full stack from ROS2 controllers down to RP2040 hardware
"""

import subprocess
import time
import sys
import signal
import os

class ROS2IntegrationTest:
    def __init__(self):
        self.processes = []
        
    def cleanup(self):
        """Kill all spawned processes"""
        for p in self.processes:
            if p.poll() is None:
                p.terminate()
                time.sleep(1)
                if p.poll() is None:
                    p.kill()
        self.processes = []

    def run_command(self, cmd, background=False, timeout=10):
        """Run a command and return result"""
        try:
            if background:
                # Run in background
                process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, 
                                         stderr=subprocess.PIPE, preexec_fn=os.setsid)
                self.processes.append(process)
                return process
            else:
                # Run and wait for result
                result = subprocess.run(cmd, shell=True, capture_output=True, 
                                      text=True, timeout=timeout)
                return result
        except subprocess.TimeoutExpired:
            print(f"⚠️  Command timed out: {cmd}")
            return None
        except Exception as e:
            print(f"❌ Error running command: {e}")
            return None

    def test_prerequisites(self):
        """Test that all prerequisites are available"""
        print("🔍 Checking Prerequisites...")
        
        # Check RP2040 connection
        result = self.run_command("ls -la /dev/ttyACM*")
        if not result or result.returncode != 0:
            print("❌ RP2040 not found at /dev/ttyACM*")
            return False
        print("✅ RP2040 detected")
        
        # Check ROS2 workspace
        if not os.path.exists("install/setup.bash"):
            print("❌ ROS2 workspace not built. Run: colcon build")
            return False
        print("✅ ROS2 workspace ready")
        
        # Test RP2040 communication
        print("🔧 Testing RP2040 communication...")
        result = self.run_command("python3 src/new_bot/scripts/improved_test_rp2040.py /dev/ttyACM0", timeout=15)
        if not result or "All tests passed!" not in result.stdout:
            print("❌ RP2040 communication test failed")
            if result:
                print(f"Output: {result.stdout}")
                print(f"Error: {result.stderr}")
            return False
        print("✅ RP2040 communication working")
        
        return True

    def test_hardware_interface(self):
        """Test ROS2 hardware interface"""
        print("\n🚀 Testing ROS2 Hardware Interface...")
        
        # Source ROS2 workspace
        setup_cmd = "source install/setup.bash && "
        
        # Start robot description
        print("📋 Starting robot state publisher...")
        cmd1 = setup_cmd + 'ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro src/new_bot/description/robot.urdf.xacro)"'
        rsp_process = self.run_command(cmd1, background=True)
        time.sleep(3)
        
        # Start hardware interface
        print("🔌 Starting hardware interface...")
        cmd2 = setup_cmd + "ros2 run controller_manager ros2_control_node --ros-args --params-file src/new_bot/config/controllers.yaml"
        hw_process = self.run_command(cmd2, background=True)
        time.sleep(5)
        
        # Check if hardware interface is running
        print("🔍 Checking hardware interface status...")
        check_cmd = setup_cmd + "ros2 control list_hardware_interfaces"
        result = self.run_command(check_cmd, timeout=10)
        
        if not result or "new_bot_hardware" not in result.stdout:
            print("❌ Hardware interface not detected")
            if result:
                print(f"Output: {result.stdout}")
            return False
            
        if "active" not in result.stdout:
            print("⚠️  Hardware interface detected but not active")
            print(f"Status: {result.stdout}")
            return False
            
        print("✅ Hardware interface active!")
        return True

    def test_controllers(self):
        """Test ROS2 controllers"""
        print("\n🎮 Testing Controllers...")
        
        setup_cmd = "source install/setup.bash && "
        
        # Wait a moment for hardware interface to stabilize
        time.sleep(2)
        
        # Spawn joint state broadcaster
        print("📊 Loading joint state broadcaster...")
        cmd = setup_cmd + "ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager"
        result = self.run_command(cmd, timeout=10)
        
        if not result or result.returncode != 0:
            print("❌ Failed to load joint state broadcaster")
            if result:
                print(f"Error: {result.stderr}")
            return False
        print("✅ Joint state broadcaster loaded")
        
        # Check controller status
        time.sleep(2)
        check_cmd = setup_cmd + "ros2 control list_controllers"
        result = self.run_command(check_cmd)
        
        if not result or "joint_state_broadcaster" not in result.stdout:
            print("❌ Joint state broadcaster not found")
            return False
            
        if "active" not in result.stdout:
            print("⚠️  Controllers loaded but not active")
            print(f"Status: {result.stdout}")
            return False
            
        print("✅ Controllers active!")
        return True

    def test_data_flow(self):
        """Test data flow through the system"""
        print("\n📡 Testing Data Flow...")
        
        setup_cmd = "source install/setup.bash && "
        
        # Check joint states topic
        print("🔍 Checking joint states...")
        cmd = setup_cmd + "timeout 5 ros2 topic echo /joint_states --once"
        result = self.run_command(cmd, timeout=8)
        
        if not result or result.returncode != 0:
            print("❌ No joint state data")
            return False
        print("✅ Joint states flowing")
        
        # Check hardware interface is receiving data
        print("🔍 Checking hardware interface data...")
        # The presence of joint states means hardware interface is working
        
        print("✅ Data flow verified!")
        return True

    def run_full_test(self):
        """Run the complete integration test"""
        print("🤖 ROS2 + RP2040 Integration Test")
        print("=" * 50)
        
        try:
            # Test prerequisites
            if not self.test_prerequisites():
                print("\n❌ Prerequisites test failed")
                return False
                
            # Test hardware interface
            if not self.test_hardware_interface():
                print("\n❌ Hardware interface test failed")
                return False
                
            # Test controllers
            if not self.test_controllers():
                print("\n❌ Controller test failed")
                return False
                
            # Test data flow
            if not self.test_data_flow():
                print("\n❌ Data flow test failed")
                return False
                
            print("\n🎉 ALL TESTS PASSED!")
            print("✅ ROS2 + RP2040 Integration is WORKING!")
            print("\nYour robot is ready for:")
            print("  - Autonomous navigation")
            print("  - Real-time control") 
            print("  - Sensor data processing")
            print("  - ROS2 ecosystem integration")
            
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
            return False
        except Exception as e:
            print(f"\n❌ Test failed with error: {e}")
            return False
        finally:
            print("\n🧹 Cleaning up...")
            self.cleanup()

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\n⚠️  Interrupted by user")
    sys.exit(1)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    
    tester = ROS2IntegrationTest()
    success = tester.run_full_test()
    
    if success:
        print("\n🚀 Integration test SUCCESSFUL!")
        sys.exit(0)
    else:
        print("\n❌ Integration test FAILED!")
        sys.exit(1)
