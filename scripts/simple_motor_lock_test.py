#!/usr/bin/env python3
"""
Simple Motor Lock Test

This script demonstrates the basic principle that Dynamixel motors 
automatically lock to their current position when torque is enabled.

Usage:
    python scripts/simple_motor_lock_test.py --fake
    python scripts/simple_motor_lock_test.py --port /dev/serial/by-id/your-device
"""

import argparse
import time
import numpy as np
import sys
import os

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gello.dynamixel.driver import DynamixelDriver, FakeDynamixelDriver


def test_basic_locking(driver):
    """Test the basic principle: torque ON = position locked"""
    
    print("🔍 Testing Basic Motor Locking Principle")
    print("=" * 45)
    
    print("\n1. Initial state:")
    print(f"   Torque enabled: {driver.torque_enabled()}")
    
    current_pos = driver.get_joints()
    print(f"   Current positions: {np.degrees(current_pos):.1f} degrees")
    
    print("\n2. Enabling torque...")
    print("   ⚡ Motor will automatically lock at current position")
    
    # Just enable torque - that's it!
    driver.set_torque_mode(True)
    
    print(f"   ✅ Torque enabled: {driver.torque_enabled()}")
    print("   🔒 Motors are now locked and will resist movement")
    
    # Verify position is still the same
    locked_pos = driver.get_joints()
    print(f"   Position after locking: {np.degrees(locked_pos):.1f} degrees")
    
    # Small position drift is normal due to sensor noise
    position_diff = np.abs(current_pos - locked_pos)
    max_diff = np.max(position_diff)
    print(f"   Position drift: {np.degrees(max_diff):.3f} degrees (normal)")
    
    print("\n3. Motor behavior while locked:")
    print("   - Internal PID controller is active")
    print("   - Motor resists external forces")
    print("   - Position is maintained automatically")
    print("   - No additional commands needed")
    
    input("\n   Try to move the motor manually (if real hardware)")
    input("   You should feel resistance. Press Enter to continue...")
    
    print("\n4. Disabling torque...")
    driver.set_torque_mode(False)
    
    print(f"   ✅ Torque disabled: {not driver.torque_enabled()}")
    print("   🔓 Motors are now free to move manually")
    
    input("\n   Motor should move freely now. Press Enter to finish...")


def test_repeated_locking(driver):
    """Test multiple lock/unlock cycles"""
    
    print("\n🔄 Testing Repeated Lock/Unlock Cycles")
    print("=" * 40)
    
    for i in range(3):
        print(f"\nCycle {i+1}:")
        
        # Get position before locking
        pos_before = driver.get_joints()
        
        # Lock
        print("   🔒 Locking...")
        driver.set_torque_mode(True)
        time.sleep(0.5)  # Brief pause
        
        # Get position after locking  
        pos_after = driver.get_joints()
        
        # Unlock
        print("   🔓 Unlocking...")
        driver.set_torque_mode(False)
        time.sleep(0.5)
        
        # Check position consistency
        diff = np.max(np.abs(pos_before - pos_after))
        print(f"   Position consistency: {np.degrees(diff):.3f} degrees drift")
    
    print("\n✅ Repeated locking test completed")


def main():
    parser = argparse.ArgumentParser(description="Simple Motor Lock Test")
    parser.add_argument("--fake", action="store_true", help="Use fake motors")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Motor port")
    parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3], 
                       help="Motor IDs to test")
    
    args = parser.parse_args()
    
    print("🤖 Simple Motor Lock Test")
    print("Testing the principle: 'Torque ON = Position Locked'")
    
    try:
        if args.fake:
            print(f"\n📱 Using fake motors (IDs: {args.ids})")
            driver = FakeDynamixelDriver(args.ids)
        else:
            print(f"\n🔌 Connecting to real motors at {args.port}")
            print(f"   Motor IDs: {args.ids}")
            driver = DynamixelDriver(args.ids, port=args.port)
        
        # Ensure motors start unlocked
        driver.set_torque_mode(False)
        
        # Run tests
        test_basic_locking(driver)
        test_repeated_locking(driver)
        
        print("\n🎉 All tests completed!")
        print("\nKey takeaway:")
        print("   Dynamixel motors automatically lock when torque is enabled")
        print("   No need to set goal positions for simple position holding")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nTroubleshooting:")
        print("- Use --fake for testing without hardware")
        print("- Check motor connections and IDs")
        print("- Verify port permissions")
    
    finally:
        try:
            # Ensure motors are unlocked on exit
            if 'driver' in locals():
                driver.set_torque_mode(False)
                print("\n🔓 Motors unlocked before exit")
        except:
            pass


if __name__ == "__main__":
    main() 