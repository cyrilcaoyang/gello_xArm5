#!/usr/bin/env python3
"""
Real xArm5 Robot Demonstration Script

This script connects your Gello device (COM5) to the real xArm5 robot at 192.168.1.237
and allows you to control the robot through teleoperation.

Usage:
    python scripts/xarm5_real_demo.py

Safety:
    - The robot will move to match your Gello device movements
    - Keep emergency stop accessible
    - Ensure the workspace is clear
"""

import sys
import os
import time
import signal
import threading
import numpy as np
from typing import Optional

# Add the project root to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n🛑 Ctrl+C received, stopping demonstration...")
    global running
    running = False

def main():
    print("🤖 xArm5 Real Robot Demonstration")
    print("================================")
    print("⚠️  SAFETY WARNING: This will control a REAL ROBOT!")
    print("⚠️  Ensure the workspace is clear and emergency stop is accessible.")
    print()
    
    # Confirm before proceeding
    response = input("Are you ready to proceed? (yes/no): ").lower().strip()
    if response not in ['yes', 'y']:
        print("❌ Demonstration cancelled.")
        return
    
    print("\n📦 Initializing components...")
    
    # Import and initialize components
    try:
        from gello.agents.gello_agent import GelloAgent
        from gello.robots.xarm5_robot import XArm5Robot
        
        print("✓ Imports successful")
        
        # Initialize Gello device
        print("🎮 Connecting to Gello device (COM5)...")
        gello = GelloAgent(port="COM5")
        print("✓ Gello device connected")
        
        # Initialize xArm5 robot
        print("🤖 Connecting to xArm5 robot (192.168.1.237)...")
        robot = XArm5Robot(ip="192.168.1.237", real=True)
        print("✓ xArm5 robot connected")
        
        # Setup signal handler
        global running
        running = True
        signal.signal(signal.SIGINT, signal_handler)
        
        print("\n🎯 Starting teleoperation...")
        print("Move your Gello device to control the robot")
        print("Press Ctrl+C to stop")
        print()
        
        # Get initial states
        initial_gello_state = gello.act({})
        initial_robot_state = robot.get_state()
        
        print(f"📊 Initial Gello state: {np.round(initial_gello_state, 3)}")
        print(f"📊 Initial Robot state: {np.round(initial_robot_state.joints(), 3)}")
        print()
        
        control_count = 0
        start_time = time.time()
        
        while running:
            try:
                # Get current Gello state
                gello_state = gello.act({})
                
                if len(gello_state) >= 6:
                    # Extract joint commands (first 5) and gripper (last 1)
                    joint_commands = gello_state[:5]
                    gripper_command = gello_state[5]
                    
                    # Send commands to robot
                    robot.set_command(joint_commands, gripper_command)
                    
                    control_count += 1
                    
                    # Print status every 50 commands
                    if control_count % 50 == 0:
                        elapsed = time.time() - start_time
                        hz = control_count / elapsed
                        
                        print(f"📈 Commands sent: {control_count} | Rate: {hz:.1f} Hz")
                        print(f"   Joint targets: [{joint_commands[0]:6.3f}, {joint_commands[1]:6.3f}, {joint_commands[2]:6.3f}, {joint_commands[3]:6.3f}, {joint_commands[4]:6.3f}]")
                        print(f"   Gripper target: {gripper_command:.3f}")
                        
                        # Get current robot state
                        current_robot_state = robot.get_state()
                        robot_joints = current_robot_state.joints()
                        print(f"   Robot joints:   [{robot_joints[0]:6.3f}, {robot_joints[1]:6.3f}, {robot_joints[2]:6.3f}, {robot_joints[3]:6.3f}, {robot_joints[4]:6.3f}]")
                        print()
                
                # Control loop frequency
                time.sleep(1.0 / 30.0)  # 30 Hz
                
            except Exception as e:
                print(f"❌ Error in control loop: {e}")
                break
                
    except KeyboardInterrupt:
        print("\n✅ Demonstration stopped by user")
        
    except Exception as e:
        print(f"❌ Error during initialization: {e}")
        print("\n🔧 Troubleshooting tips:")
        print("1. Check that the xArm5 robot is powered on")
        print("2. Verify network connection to 192.168.1.237")
        print("3. Ensure Gello device is connected to COM5")
        print("4. Check that no other program is using the devices")
        
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        try:
            if 'robot' in locals():
                print("🤖 Stopping robot...")
                robot.stop()
                print("✓ Robot stopped")
        except Exception as e:
            print(f"⚠️  Error stopping robot: {e}")
            
        print("✅ Cleanup complete")

if __name__ == "__main__":
    main()
