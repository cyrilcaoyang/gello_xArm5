#!/usr/bin/env python3
"""
Test connection to the real xArm5 robot at 192.168.1.237

This script tests the basic connection and reads the robot state
without moving the robot, except the gripper.
"""

import sys
import os
import time
import signal

# Add the project root to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Global flag for clean exit
running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global running
    running = False
    print("\n🛑 Interrupted by user")
    sys.exit(0)

def test_robot_connection():
    global running
    
    # Setup signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    print("🤖 xArm5 Connection Test")
    print("========================")
    print("Testing connection to xArm5 robot at 192.168.1.237")
    print("This test will NOT move the robot - only read its status.")
    print("Press Ctrl+C to interrupt at any time.\n")
    
    try:
        print("📦 Importing xArm5Robot...")
        from gello.robots.xarm5_robot import XArm5Robot
        print("✓ Import successful")
        
        print("🔌 Attempting to connect to robot...")
        print("⏳ This may take a few seconds...")
        
        try:
            robot = XArm5Robot(ip="192.168.1.237", real=True)
            print("✓ Robot object created")
            
            # Test if we can actually communicate with the robot
            print("🔍 Testing robot communication...")
            time.sleep(2)  # Give it a moment to initialize
            
            # Try to get initial state with timeout - this will fail if robot is not actually connected
            print("⏳ Testing robot state reading (this will timeout if robot is unreachable)...")
            start_time = time.time()
            timeout = 10  # 10 second timeout
            
            while time.time() - start_time < timeout:
                if not running:
                    return False
                try:
                    test_state = robot.get_state()
                    print("✓ Robot communication successful!")
                    break
                except Exception as comm_error:
                    if "not connect" in str(comm_error):
                        time.sleep(0.5)  # Wait a bit and retry
                        continue
                    else:
                        raise comm_error
            else:
                # Timeout reached
                raise Exception("Robot communication timeout - robot may not be accessible")
            
        except Exception as init_error:
            print(f"❌ Robot initialization failed: {init_error}")
            print("⚠️  Robot may not be reachable at 192.168.1.237")
            return False
        
        print("📊 Reading initial robot state...")
        
        # Read initial state
        state = robot.get_state()
        joints = state.joints()
        cartesian = state.cartesian_pos()
        gripper = state.gripper_pos()
        
        print(f"Initial state:")
        print(f"  Joints (rad): [{joints[0]:7.4f}, {joints[1]:7.4f}, {joints[2]:7.4f}, {joints[3]:7.4f}, {joints[4]:7.4f}]")
        print(f"  Joints (deg): [{np.rad2deg(joints[0]):7.1f}, {np.rad2deg(joints[1]):7.1f}, {np.rad2deg(joints[2]):7.1f}, {np.rad2deg(joints[3]):7.1f}, {np.rad2deg(joints[4]):7.1f}]")
        print(f"  Cartesian:    [{cartesian[0]:7.1f}, {cartesian[1]:7.1f}, {cartesian[2]:7.1f}] mm")
        print(f"  Gripper:      {gripper:7.3f}")
        
        print("\n🤏 Testing gripper movement...")
        print("This will open and close the gripper to verify robot control.")
        
        # Test gripper open/close sequence
        gripper_positions = [
            (1.0, "OPEN"),
            (0.0, "CLOSE"), 
            (1.0, "OPEN")
        ]
        
        for i, (gripper_cmd, description) in enumerate(gripper_positions):
            if not running:
                print("🛑 Test interrupted")
                break
                
            print(f"\n--- Gripper Test {i+1}/3: {description} (command: {gripper_cmd}) ---")
            
            try:
                # Send gripper command (keep joints at current position)
                robot.set_command(joints, gripper_cmd)
                
                # Wait for movement with check for interruption
                for _ in range(20):  # 2 seconds in 0.1s increments
                    if not running:
                        break
                    time.sleep(0.1)
                
                if not running:
                    break
                
                # Read current state
                current_state = robot.get_state()
                current_gripper = current_state.gripper_pos()
                current_joints = current_state.joints()
                
                print(f"  Gripper position: {current_gripper:7.3f}")
                print(f"  Joints (deg):     [{np.rad2deg(current_joints[0]):7.1f}, {np.rad2deg(current_joints[1]):7.1f}, {np.rad2deg(current_joints[2]):7.1f}, {np.rad2deg(current_joints[3]):7.1f}, {np.rad2deg(current_joints[4]):7.1f}]")
                
                if i < len(gripper_positions) - 1:
                    print("  Waiting...")
                    # Short wait with interruption check
                    for _ in range(10):  # 1 second in 0.1s increments
                        if not running:
                            break
                        time.sleep(0.1)
                        
            except Exception as e:
                print(f"⚠️  Error during gripper test: {e}")
                break
        
        print(f"\n✅ SUCCESS: Robot is responding normally!")
        print("The robot is ready for teleoperation.")
        
    except Exception as e:
        print(f"❌ ERROR: {e}")
        print("\n🔧 Troubleshooting checklist:")
        print("1. Is the xArm5 robot powered on?")
        print("2. Is the robot connected to the network?")
        print("3. Can you ping 192.168.1.237?")
        print("4. Is the robot in the correct mode (not in emergency stop)?")
        print("5. Are there any error lights on the robot?")
        return False
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        return False
        
    finally:
        try:
            if 'robot' in locals():
                print("\n🧹 Stopping robot...")
                robot.stop()
                print("✓ Robot stopped cleanly")
        except Exception as e:
            print(f"⚠️  Error during robot cleanup: {e}")
    
    return True

if __name__ == "__main__":
    import numpy as np
    test_robot_connection()
