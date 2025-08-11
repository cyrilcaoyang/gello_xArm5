#!/usr/bin/env python3
"""
Read values from Gello device to understand current joint positions.
This helps debug joint mapping issues.
"""

import sys
import os
import time
import numpy as np

# Add the project root to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gello.agents.gello_agent import GelloAgent

def main():
    print("Reading Gello Device Values")
    print("===========================")
    
    try:
        # Create Gello agent with COM5 (uses your tested configuration)
        agent = GelloAgent(port="COM5")
        print("✓ Connected to Gello device on COM5")
        
        print("\nReading joint values (move your Gello device):")
        print("Joint order: [J1, J2, J3, J4, J5, Gripper]")
        print("Press Ctrl+C to stop\n")
        
        while True:
            # Get current joint state from Gello
            joint_state = agent.act({})  # Empty obs dict
            
            # Convert to degrees for easier reading
            joints_deg = np.rad2deg(joint_state[:5])  # First 5 are joints
            gripper = joint_state[5]  # Last one is gripper
            
            print(f"Joints (deg): [{joints_deg[0]:6.1f}, {joints_deg[1]:6.1f}, {joints_deg[2]:6.1f}, {joints_deg[3]:6.1f}, {joints_deg[4]:6.1f}] | Gripper: {gripper:5.2f}", end="\r")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopped reading Gello values.")
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure:")
        print("1. Gello device is connected to COM5")
        print("2. No other program is using the device")

if __name__ == "__main__":
    main()
