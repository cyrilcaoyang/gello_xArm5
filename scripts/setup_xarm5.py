#!/usr/bin/env python3
"""
xArm5 Setup and Quick Start Script

This script helps you get started with xArm5 robot quickly by:
1. Verifying your setup
2. Testing basic functionality
3. Providing example configurations
4. Running interactive setup

Usage:
    python scripts/setup_xarm5.py --help
    python scripts/setup_xarm5.py --test-sim
    python scripts/setup_xarm5.py --test-real --ip 192.168.1.226
    python scripts/setup_xarm5.py --interactive
"""

import argparse
import sys
import time
import subprocess
from pathlib import Path


def check_dependencies():
    """Check if required packages are installed."""
    print("🔍 Checking dependencies...")
    
    required_packages = [
        'numpy',
        'xarm',
        'pyquaternion'
    ]
    
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package)
            print(f"  ✅ {package}")
        except ImportError:
            print(f"  ❌ {package} - MISSING")
            missing_packages.append(package)
    
    if missing_packages:
        print(f"\n⚠️  Missing packages: {', '.join(missing_packages)}")
        print("   Install with: pip install " + " ".join(missing_packages))
        return False
    
    print("✅ All dependencies found!")
    return True


def test_import():
    """Test if xArm5Robot can be imported."""
    print("\n🔍 Testing xArm5Robot import...")
    
    try:
        from gello.robots.xarm5_robot import XArm5Robot
        print("✅ XArm5Robot imported successfully!")
        return True
    except ImportError as e:
        print(f"❌ Failed to import XArm5Robot: {e}")
        return False


def test_simulation():
    """Test xArm5 in simulation mode."""
    print("\n🔍 Testing xArm5 in simulation mode...")
    
    try:
        from gello.robots.xarm5_robot import XArm5Robot
        
        # Create robot in simulation mode
        robot = XArm5Robot(real=False)
        
        print(f"✅ Robot created with {robot.num_dofs()} DOFs")
        
        # Test basic operations
        state = robot.get_joint_state()
        print(f"✅ Joint state: {state}")
        
        # Test command
        import numpy as np
        test_command = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.0])
        robot.command_joint_state(test_command)
        print("✅ Joint command executed")
        
        robot.stop()
        print("✅ Simulation test completed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Simulation test failed: {e}")
        return False


def test_real_robot(ip):
    """Test connection to real xArm5 robot."""
    print(f"\n🔍 Testing connection to real xArm5 at {ip}...")
    
    try:
        from gello.robots.xarm5_robot import XArm5Robot
        
        # Create robot connection
        robot = XArm5Robot(ip=ip, real=True)
        
        print(f"✅ Connected to robot with {robot.num_dofs()} DOFs")
        
        # Test basic operations
        state = robot.get_joint_state()
        print(f"✅ Joint state: {state}")
        
        robot.stop()
        print("✅ Real robot test completed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Real robot test failed: {e}")
        print("   Make sure:")
        print("   - Robot is powered on")
        print("   - IP address is correct")
        print("   - Network connection is established")
        print("   - xArm Python SDK is installed")
        return False


def generate_gello_config():
    """Generate example GELLO configuration."""
    print("\n📝 Example GELLO Configuration for xArm5:")
    print("Add this to gello/agents/gello_agent.py in the PORT_CONFIG_MAP:")
    print()
    
    config = '''
# xArm5 (5 DOF) - Example configuration
"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_YOUR_ID": DynamixelRobotConfig(
    joint_ids=(1, 2, 3, 4, 5),
    joint_offsets=(
        2 * np.pi / 2,
        2 * np.pi / 2,
        2 * np.pi / 2,
        2 * np.pi / 2,
        -1 * np.pi / 2 + 2 * np.pi,
    ),
    joint_signs=(1, 1, 1, 1, 1),
    gripper_config=(6, 279, 229),  # Adjust gripper values as needed
),
'''
    
    print(config)
    print("Remember to:")
    print("1. Replace YOUR_ID with your actual device ID")
    print("2. Calibrate joint offsets using: python scripts/gello_get_offset.py")
    print("3. Adjust joint signs if needed: (1, 1, 1, 1, 1)")


def interactive_setup():
    """Run interactive setup process."""
    print("\n🚀 xArm5 Interactive Setup")
    print("=" * 40)
    
    # Step 1: Check dependencies
    if not check_dependencies():
        print("\n❌ Please install missing dependencies first.")
        return False
    
    # Step 2: Test import
    if not test_import():
        print("\n❌ Import test failed. Check your installation.")
        return False
    
    # Step 3: Choose test mode
    print("\n📋 Choose test mode:")
    print("1. Test simulation only")
    print("2. Test real robot")
    print("3. Generate GELLO configuration")
    print("4. All of the above")
    
    choice = input("Enter choice (1-4): ").strip()
    
    success = True
    
    if choice in ['1', '4']:
        success &= test_simulation()
    
    if choice in ['2', '4']:
        ip = input("Enter robot IP address [192.168.1.226]: ").strip()
        if not ip:
            ip = "192.168.1.226"
        success &= test_real_robot(ip)
    
    if choice in ['3', '4']:
        generate_gello_config()
    
    if success:
        print("\n🎉 Setup completed successfully!")
        print("\nNext steps:")
        print("1. Run simulation: python experiments/launch_nodes.py --robot sim_xarm5")
        print("2. Try examples: python scripts/xarm5_example.py --sim")
        print("3. Configure GELLO if needed")
        print("4. Read the updated README for more information")
    else:
        print("\n⚠️  Setup completed with some issues. Check the errors above.")
    
    return success


def main():
    parser = argparse.ArgumentParser(description="xArm5 Setup and Testing Script")
    parser.add_argument("--test-sim", action="store_true", help="Test simulation mode")
    parser.add_argument("--test-real", action="store_true", help="Test real robot")
    parser.add_argument("--ip", type=str, default="192.168.1.226", help="Robot IP address")
    parser.add_argument("--interactive", action="store_true", help="Run interactive setup")
    parser.add_argument("--check-deps", action="store_true", help="Check dependencies only")
    parser.add_argument("--gello-config", action="store_true", help="Show GELLO configuration")
    
    args = parser.parse_args()
    
    if len(sys.argv) == 1:
        # No arguments provided, run interactive mode
        args.interactive = True
    
    print("🤖 xArm5 Setup Script")
    print("=" * 30)
    
    if args.check_deps:
        check_dependencies()
        return
    
    if args.gello_config:
        generate_gello_config()
        return
    
    if args.interactive:
        interactive_setup()
        return
    
    success = True
    
    if args.test_sim:
        success &= check_dependencies()
        success &= test_import()
        success &= test_simulation()
    
    if args.test_real:
        success &= check_dependencies()
        success &= test_import()
        success &= test_real_robot(args.ip)
    
    if success:
        print("\n✅ All tests passed!")
    else:
        print("\n❌ Some tests failed. Check the output above.")
        sys.exit(1)


if __name__ == "__main__":
    main()