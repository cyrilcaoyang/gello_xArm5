#!/usr/bin/env python3
"""
xArm5 + Gello Real Robot Demonstration

This script connects your Gello device to control the real xArm5 robot.
Since the gripper test worked, we know the robot connection is good!
"""

import sys
import os
import time
import signal
import numpy as np

# Add the project root to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Global flag for clean exit
running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global running
    running = False
    print("\n🛑 Stopping demonstration...")

def main():
    global running
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("🎮🤖 xArm5 + Gello Real Robot Demo")
    print("=================================")
    print("⚠️  SAFETY: Make sure workspace is clear!")
    print("⚠️  Keep emergency stop accessible!")
    print("🎯 Move your Gello device to control the robot")
    print("🛑 Press Ctrl+C to stop anytime\n")
    
    # Confirm before proceeding
    response = input("Ready to start? (yes/no): ").lower().strip()
    if response not in ['yes', 'y']:
        print("❌ Demo cancelled.")
        return
    
    robot = None
    gello = None
    
    try:
        print("\n📦 Initializing Gello device...")
        from gello.agents.gello_agent import GelloAgent
        # Use direct configuration instead of config files
        gello = GelloAgent(
            port="COM5",
            start_joints=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 5 joints + gripper
        )
        print("✓ Gello device ready")
        
        print("🤖 Initializing xArm5 robot...")
        from gello.robots.xarm5_robot import XArm5Robot
        robot = XArm5Robot(ip="192.168.1.237", real=True)
        print("✓ Robot ready")
        
        print("\n🎯 Starting teleoperation...")
        print("Move your Gello device now!")
        
        control_count = 0
        start_time = time.time()
        
        while running:
            # Get Gello state
            gello_state = gello.act({})
            
            if len(gello_state) >= 6:
                # Send to robot
                joints = gello_state[:5]  # 5 joints
                gripper = gello_state[5]  # gripper
                
                robot.set_command(joints, gripper)
                control_count += 1
                
                # Status update every 2 seconds
                if control_count % 60 == 0:  # 30 Hz * 2 seconds
                    elapsed = time.time() - start_time
                    rate = control_count / elapsed
                    print(f"📈 Commands: {control_count} | Rate: {rate:.1f} Hz | Time: {elapsed:.1f}s")
            
            # Control frequency
            time.sleep(1.0 / 30.0)  # 30 Hz
            
    except KeyboardInterrupt:
        print("\n✅ Demo stopped by user")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        
    finally:
        print("\n🧹 Cleaning up...")
        
        # Stop robot first (most important)
        if robot is not None:
            try:
                print("🤖 Stopping robot...")
                robot.stop()
                print("✓ Robot stopped")
            except Exception as e:
                print(f"⚠️  Error stopping robot: {e}")
        
        # Clean up Gello
        if gello is not None:
            try:
                print("🎮 Disconnecting Gello...")
                # GelloAgent doesn't need explicit cleanup
                print("✓ Gello disconnected")
            except Exception as e:
                print(f"⚠️  Error with Gello: {e}")
        
        print("✅ Demo ended safely")

if __name__ == "__main__":
    main()
