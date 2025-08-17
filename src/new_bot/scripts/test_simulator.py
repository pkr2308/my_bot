#!/usr/bin/env python3

"""
Quick test of the RP2040 simulator
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rp2040_simulator import RP2040Simulator
import time

def test_simulator():
    print("=== Testing RP2040 Simulator ===\n")
    
    sim = RP2040Simulator()
    sim.connect()
    
    # Test basic commands
    commands = [
        "PING",
        "INIT", 
        "READ",
        "CMD,P:0.5,P:-0.3,V:1.0,V:0.0",
        "READ",
        "STOP",
        "READ"
    ]
    
    for cmd in commands:
        print(f"Testing command: {cmd}")
        response = sim.parse_command(cmd)
        print(f"Response: {response}")
        
        # Update simulation between commands
        for i in range(10):
            sim.update_simulation()
            time.sleep(0.01)
        
        print()
    
    sim.disconnect()
    print("✓ Simulator test completed successfully!")

if __name__ == '__main__':
    test_simulator()
