#!/usr/bin/env python3
"""
Example script demonstrating xArm5 robot usage.

This script shows how to:
1. Initialize an xArm5 robot (5 DOF + gripper)
2. Control individual joints
3. Use GELLO for teleoperation
4. Basic robot operations

Usage:
    # For simulation
    python scripts/xarm5_example.py --sim

    # For real robot (make sure robot is connected and IP is correct)
    python scripts/xarm5_example.py --real --ip 192.168.1.226

    # Launch with GELLO support
    python scripts/xarm5_example.py --gello
"""

import argparse
import time
import numpy as np
from dataclasses import dataclass
from typing import Optional


def demo_basic_movements(robot):
    """Demonstrate basic robot movements."""
    print("Demo: Basic movements")
    
    # Get current state
    current_state = robot.get_joint_state()
    print(f"Current joint state: {current_state}")
    print(f"Number of DOFs: {robot.num_dofs()}")
    
    # Demo 1: Move individual joints
    print("\n1. Moving individual joints...")
    
    # Create target positions (5 joints + gripper)
    target_joints = np.array([0.0, 0.2, -0.3, 0.1, 0.0, 0.5])  # 5 joints + gripper
    
    print(f"Moving to target: {target_joints}")
    robot.command_joint_state(target_joints)
    time.sleep(3)
    
    # Demo 2: Open and close gripper
    print("\n2. Gripper control...")
    
    # Open gripper (last element is gripper position: 0 = closed, 1 = open)
    gripper_open = np.copy(target_joints)
    gripper_open[-1] = 0.0  # Open gripper
    print("Opening gripper...")
    robot.command_joint_state(gripper_open)
    time.sleep(2)
    
    # Close gripper
    gripper_closed = np.copy(target_joints)
    gripper_closed[-1] = 1.0  # Close gripper
    print("Closing gripper...")
    robot.command_joint_state(gripper_closed)
    time.sleep(2)
    
    # Demo 3: Return to home position
    print("\n3. Returning to home position...")
    home_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # All joints at 0
    robot.command_joint_state(home_position)
    time.sleep(3)
    
    print("Basic movements demo completed!")


def demo_trajectory(robot):
    """Demonstrate a simple trajectory."""
    print("\nDemo: Simple trajectory")
    
    # Define a simple sinusoidal trajectory for joint 2
    duration = 10.0  # seconds
    frequency = 0.5  # Hz
    amplitude = 0.3  # radians
    
    start_time = time.time()
    home_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    print(f"Executing {duration}s sinusoidal trajectory on joint 2...")
    
    while time.time() - start_time < duration:
        elapsed = time.time() - start_time
        
        # Calculate target position for joint 2
        joint2_pos = amplitude * np.sin(2 * np.pi * frequency * elapsed)
        
        # Create target joint state
        target = np.copy(home_position)
        target[1] = joint2_pos  # Joint 2 (index 1)
        
        robot.command_joint_state(target)
        time.sleep(0.05)  # 20 Hz control loop
    
    # Return to home
    robot.command_joint_state(home_position)
    print("Trajectory demo completed!")


def run_with_gello():
    """Launch robot and GELLO nodes for teleoperation."""
    print("Launching xArm5 with GELLO teleoperation...")
    print("This will start the robot server and GELLO client.")
    print("Make sure your GELLO device is connected!")
    
    import subprocess
    import os
    from multiprocessing import Process
    
    # Launch robot server
    def launch_robot():
        os.system("python experiments/launch_nodes.py --robot xarm5")
    
    # Launch GELLO client
    def launch_gello():
        time.sleep(2)  # Wait for robot server to start
        os.system("python experiments/run_env.py --agent=gello")
    
    robot_process = Process(target=launch_robot)
    gello_process = Process(target=launch_gello)
    
    try:
        robot_process.start()
        gello_process.start()
        
        print("Both processes started. Press Ctrl+C to stop.")
        robot_process.join()
        gello_process.join()
        
    except KeyboardInterrupt:
        print("\nShutting down...")
        robot_process.terminate()
        gello_process.terminate()
        robot_process.join()
        gello_process.join()


def main():
    parser = argparse.ArgumentParser(description="xArm5 Robot Example")
    parser.add_argument("--sim", action="store_true", help="Use simulation mode")
    parser.add_argument("--real", action="store_true", help="Use real robot")
    parser.add_argument("--ip", type=str, default="192.168.1.226", help="Robot IP address")
    parser.add_argument("--gello", action="store_true", help="Launch with GELLO teleoperation")
    parser.add_argument("--demo", choices=["basic", "trajectory", "all"], default="all", 
                       help="Choose demo type")
    
    args = parser.parse_args()
    
    if args.gello:
        run_with_gello()
        return
    
    # Import robot class
    from gello.robots.xarm5_robot import XArm5Robot
    
    # Initialize robot
    if args.sim:
        print("Initializing xArm5 in simulation mode...")
        robot = XArm5Robot(ip=args.ip, real=False)
    elif args.real:
        print(f"Initializing xArm5 robot at IP: {args.ip}")
        robot = XArm5Robot(ip=args.ip, real=True)
    else:
        print("Please specify either --sim or --real mode")
        return
    
    try:
        print("xArm5 Robot initialized successfully!")
        print(f"Robot has {robot.num_dofs()} DOFs (5 joints + 1 gripper)")
        
        # Run selected demos
        if args.demo in ["basic", "all"]:
            demo_basic_movements(robot)
        
        if args.demo in ["trajectory", "all"]:
            demo_trajectory(robot)
            
        print("\nAll demos completed successfully!")
        
    except Exception as e:
        print(f"Error during demo: {e}")
        
    finally:
        print("Stopping robot...")
        robot.stop()
        print("Robot stopped.")


if __name__ == "__main__":
    main()