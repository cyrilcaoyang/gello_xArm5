#!/usr/bin/env python3
"""
GELLO Position Locking Demo

This script demonstrates the new position locking functionality for GELLO devices.
It shows how to:
1. Read current GELLO positions
2. Lock motors at current position
3. Lock motors at specific target positions  
4. Unlock motors for manual movement
5. Interactive control for testing

Usage:
    # Interactive mode (recommended)
    python scripts/gello_position_lock_demo.py --interactive

    # Auto demo
    python scripts/gello_position_lock_demo.py --port /dev/serial/by-id/your-device-id

    # Test with fake hardware
    python scripts/gello_position_lock_demo.py --fake

Requirements:
    - GELLO device connected (or --fake for testing)
    - Proper GELLO configuration in gello/agents/gello_agent.py
"""

import argparse
import time
import numpy as np
from typing import Optional
import sys
import os

# Add the project root to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gello.agents.gello_agent import PORT_CONFIG_MAP
from gello.robots.dynamixel import DynamixelRobot


def find_gello_port():
    """Try to find a configured GELLO port automatically."""
    available_ports = list(PORT_CONFIG_MAP.keys())
    
    if not available_ports:
        return None
    
    print("Available configured GELLO ports:")
    for i, port in enumerate(available_ports):
        print(f"  {i+1}. {port}")
    
    if len(available_ports) == 1:
        return available_ports[0]
    
    try:
        choice = int(input(f"Select port (1-{len(available_ports)}): ")) - 1
        return available_ports[choice]
    except (ValueError, IndexError):
        return available_ports[0]


def create_gello_robot(port: Optional[str] = None, fake: bool = False) -> DynamixelRobot:
    """Create and initialize GELLO robot."""
    
    if fake:
        print("🤖 Creating fake GELLO for testing...")
        # Create a fake GELLO with common configuration
        return DynamixelRobot(
            joint_ids=(1, 2, 3, 4, 5, 6, 7),
            joint_offsets=np.zeros(7),
            joint_signs=np.ones(7),
            real=False,
            gripper_config=(8, 270, 220)
        )
    
    if port is None:
        port = find_gello_port()
        if port is None:
            raise ValueError("No GELLO configuration found. Add your device to PORT_CONFIG_MAP in gello/agents/gello_agent.py")
    
    if port not in PORT_CONFIG_MAP:
        raise ValueError(f"Port {port} not configured. Available: {list(PORT_CONFIG_MAP.keys())}")
    
    config = PORT_CONFIG_MAP[port]
    print(f"🤖 Connecting to GELLO at {port}")
    print(f"   Joints: {len(config.joint_ids)} DOF")
    
    return config.make_robot(port=port)


def demo_basic_locking(gello: DynamixelRobot):
    """Demonstrate basic position locking functionality."""
    print("\n" + "="*50)
    print("🔒 BASIC POSITION LOCKING DEMO")
    print("="*50)
    
    print("\n1. Current GELLO status:")
    print(f"   {gello.get_lock_status()}")
    
    current_joints = gello.get_joint_state()
    print(f"   Current positions: {np.degrees(current_joints):.1f} degrees")
    
    print("\n2. Locking at current position...")
    gello.lock_current_position()
    
    print("   Try to move GELLO now - it should resist movement!")
    input("   Press Enter when ready to continue...")
    
    print("\n3. Unlocking GELLO...")
    gello.unlock_position()
    print(f"   {gello.get_lock_status()}")
    
    print("   You can now move GELLO freely again")
    input("   Move GELLO and press Enter to continue...")


def demo_target_locking(gello: DynamixelRobot):
    """Demonstrate locking at specific target positions."""
    print("\n" + "="*50) 
    print("🎯 TARGET POSITION LOCKING DEMO")
    print("="*50)
    
    # Define some interesting target positions
    num_joints = len(gello._joint_ids)
    if gello.gripper_open_close is not None:
        num_joints -= 1  # Exclude gripper
    
    targets = {
        "Home": np.zeros(num_joints),
        "Raised": np.array([0, -0.5, 0.5, -0.5, 0, 0, 0])[:num_joints],
        "Side": np.array([1.57, 0, 0, 0, 0, 0, 0])[:num_joints],
    }
    
    for name, target in targets.items():
        print(f"\n📍 Moving to '{name}' position: {np.degrees(target):.1f} degrees")
        
        try:
            gello.lock_at_joint_angles(target)
            print("   GELLO should move to this position and lock")
            input("   Press Enter to continue...")
            
        except Exception as e:
            print(f"   ⚠️  Error: {e}")
            continue
    
    print("\n🔓 Unlocking for free movement...")
    gello.unlock_position()


def interactive_control(gello: DynamixelRobot):
    """Interactive control interface."""
    print("\n" + "="*50)
    print("🎮 INTERACTIVE GELLO CONTROL")
    print("="*50)
    print("Commands:")
    print("  'lock'     - Lock at current position")
    print("  'unlock'   - Unlock for free movement") 
    print("  'status'   - Show current status")
    print("  'pos'      - Show current joint positions")
    print("  'home'     - Lock at home position (all zeros)")
    print("  'help'     - Show this help")
    print("  'quit'     - Exit")
    print()
    
    while True:
        try:
            cmd = input("🤖 GELLO> ").strip().lower()
            
            if cmd in ['quit', 'exit', 'q']:
                break
                
            elif cmd == 'lock':
                gello.lock_current_position()
                
            elif cmd == 'unlock':
                gello.unlock_position()
                
            elif cmd == 'status':
                print(f"Status: {gello.get_lock_status()}")
                
            elif cmd == 'pos':
                joints = gello.get_joint_state()
                print(f"Joints: {np.degrees(joints):.1f} degrees")
                print(f"Radians: {joints:.3f}")
                
            elif cmd == 'home':
                num_joints = len(gello._joint_ids)
                if gello.gripper_open_close is not None:
                    num_joints -= 1
                home_pos = np.zeros(num_joints)
                gello.lock_at_joint_angles(home_pos)
                
            elif cmd == 'help':
                print("Commands: lock, unlock, status, pos, home, help, quit")
                
            elif cmd == '':
                continue
                
            else:
                print(f"Unknown command: '{cmd}'. Type 'help' for commands.")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")


def safety_check(gello: DynamixelRobot):
    """Perform safety check before demos."""
    print("🔍 Safety Check:")
    print("   - Make sure GELLO is in a safe position")
    print("   - Ensure no obstacles around GELLO")
    print("   - Be ready to power off if needed")
    
    if not gello._torque_on:
        print("   ✅ Motors are unlocked (safe for manual movement)")
    else:
        print("   ⚠️  Motors are currently locked")
        
    response = input("Continue with demo? (y/N): ").strip().lower()
    return response in ['y', 'yes']


def main():
    parser = argparse.ArgumentParser(description="GELLO Position Locking Demo")
    parser.add_argument("--port", type=str, help="GELLO device port")
    parser.add_argument("--fake", action="store_true", help="Use fake hardware for testing")
    parser.add_argument("--interactive", action="store_true", help="Run interactive control mode")
    parser.add_argument("--demo", choices=["basic", "target", "all"], default="all", 
                       help="Which demo to run")
    
    args = parser.parse_args()
    
    try:
        # Create GELLO robot
        gello = create_gello_robot(port=args.port, fake=args.fake)
        
        print(f"✅ GELLO connected successfully!")
        print(f"   DOF: {gello.num_dofs()}")
        print(f"   Status: {gello.get_lock_status()}")
        
        if not args.fake and not safety_check(gello):
            print("Demo cancelled.")
            return
        
        if args.interactive:
            interactive_control(gello)
        else:
            if args.demo in ["basic", "all"]:
                demo_basic_locking(gello)
            
            if args.demo in ["target", "all"]:
                demo_target_locking(gello)
            
            print("\n✅ Demo completed!")
            
        # Ensure unlocked before exit
        if gello.is_locked():
            print("\n🔓 Unlocking GELLO before exit...")
            gello.unlock_position()
            
    except KeyboardInterrupt:
        print("\nDemo interrupted.")
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nTroubleshooting:")
        print("1. Check GELLO device connection")
        print("2. Verify port configuration in gello/agents/gello_agent.py")
        print("3. Try --fake flag for testing without hardware")
    
    finally:
        print("Demo finished.")


if __name__ == "__main__":
    main() 