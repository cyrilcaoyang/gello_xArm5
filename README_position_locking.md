# GELLO Position Locking Feature

This document describes the new position locking functionality added to the GELLO system, which allows you to power the Dynamixel motors and lock them at specific positions.

## Overview

**Position Locking** enables you to:
- **Lock GELLO at current position**: Powers motors to hold current joint angles
- **Lock at specific target positions**: Move GELLO to and lock at desired poses  
- **Unlock for free movement**: Return to normal teleoperation mode
- **Check lock status**: Monitor whether motors are powered or not

This is useful for:
- Holding a specific pose during robot operations
- Preventing accidental movement during calibration
- Creating stable reference positions
- Testing and debugging GELLO configurations

## API Reference

### DynamixelDriver Level

```python
from gello.dynamixel.driver import DynamixelDriver

driver = DynamixelDriver(joint_ids=[1,2,3,4,5,6,7])

# Lock at current position
driver.lock_current_position()

# Lock at specific positions (radians)
driver.lock_at_position([0, -1.57, 1.57, 0, 0, 0, 0])

# Unlock motors
driver.unlock_position()

# Check status
is_locked = driver.is_locked()
```

### DynamixelRobot Level (Recommended)

```python
from gello.robots.dynamixel import DynamixelRobot
from gello.agents.gello_agent import PORT_CONFIG_MAP

# Create GELLO robot from configuration
config = PORT_CONFIG_MAP["/dev/serial/by-id/your-device-id"]
gello = config.make_robot(port="/dev/serial/by-id/your-device-id")

# Lock at current position
gello.lock_current_position()

# Lock at specific joint angles (robot coordinate system)
target_joints = np.array([0, -1.57, 1.57, -1.57, 0, 0, 0])  # 7 DOF example
gello.lock_at_joint_angles(target_joints)

# Unlock for teleoperation
gello.unlock_position()

# Check status
print(gello.get_lock_status())  # Human-readable status
is_locked = gello.is_locked()   # Boolean
```

## Usage Examples

### Basic Locking

```python
# Connect to GELLO
gello = create_gello_robot()

# Read current position
current_pos = gello.get_joint_state()
print(f"Current position: {np.degrees(current_pos)} degrees")

# Lock at current position
gello.lock_current_position()
# Motors are now powered and resist movement

# Unlock for free movement
gello.unlock_position()
# Can move GELLO manually again
```

### Locking at Specific Positions

```python
# Define target positions
home_position = np.zeros(7)  # All joints at 0
raised_position = np.array([0, -0.5, 0.5, -0.5, 0, 0, 0])

# Move to and lock at home
gello.lock_at_joint_angles(home_position)

# Wait or do other operations
time.sleep(5)

# Move to raised position
gello.lock_at_joint_angles(raised_position)

# Unlock when done
gello.unlock_position()
```

### Integration with Teleoperation

```python
# Start teleoperation session
gello.unlock_position()  # Ensure free movement

# User moves GELLO manually
# ... teleoperation code ...

# Lock current pose for robot operation
gello.lock_current_position()

# Execute robot commands while GELLO is locked
# ... robot control code ...

# Return to teleoperation
gello.unlock_position()
```

## Demo Scripts

### Interactive Demo
```bash
# Run interactive control interface
python scripts/gello_position_lock_demo.py --interactive

# Commands available:
# lock     - Lock at current position
# unlock   - Unlock for free movement
# status   - Show current status
# pos      - Show joint positions
# home     - Lock at home position
# quit     - Exit
```

### Automated Demo
```bash
# Run full automated demo
python scripts/gello_position_lock_demo.py --port /dev/serial/by-id/your-device-id

# Test with fake hardware
python scripts/gello_position_lock_demo.py --fake

# Run specific demo parts
python scripts/gello_position_lock_demo.py --demo basic
python scripts/gello_position_lock_demo.py --demo target
```

## Technical Details

### How It Works

1. **Position Reading**: Continuously reads current motor positions
2. **Torque Control**: Enables/disables motor torque via Dynamixel SDK
3. **Position Commands**: Sets goal positions to current positions for locking
4. **Coordinate Mapping**: Handles joint offsets and signs automatically

### Safety Considerations

- **Always unlock before manual movement** - Locked motors will resist movement and may cause strain
- **Monitor motor temperature** - Prolonged locking under load can cause heating
- **Use appropriate torque limits** - Configure motor torque limits for safety
- **Emergency stop** - Be ready to power off motors if needed

### Motor States

| State | Torque | Manual Movement | Position Held |
|-------|--------|-----------------|---------------|
| **Unlocked** | Disabled | ✅ Free | ❌ No |
| **Locked** | Enabled | ❌ Resisted | ✅ Yes |

### Configuration Requirements

Position locking works with any properly configured GELLO device. Ensure your device is added to `PORT_CONFIG_MAP` in `gello/agents/gello_agent.py`:

```python
PORT_CONFIG_MAP = {
    "/dev/serial/by-id/your-device-id": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),  # Your motor IDs
        joint_offsets=(...),               # Calibrated offsets  
        joint_signs=(...),                 # Direction signs
        gripper_config=(...),              # Optional gripper
    ),
}
```

## Troubleshooting

### Common Issues

**Motors don't lock:**
- Check motor power supply
- Verify motor IDs and connections
- Check torque enable in Dynamixel Wizard

**High motor temperature:**
- Reduce locking duration
- Check motor load and torque limits
- Ensure adequate cooling

**Position drift while locked:**
- Check motor torque settings
- Verify stable power supply
- Calibrate motor positions

**Communication errors:**
- Check USB connection
- Verify baudrate settings
- Try reconnecting device

### Debug Commands

```python
# Check motor communication
gello._driver.torque_enabled()

# Read raw motor positions
raw_positions = gello._driver.get_joints()

# Check motor IDs
print(gello._joint_ids)

# Test individual motor torque
gello._driver.set_torque_mode(True)
gello._driver.set_torque_mode(False)
```

## Integration with Existing Code

The position locking feature is fully backward compatible. Existing teleoperation code continues to work unchanged. Simply add locking calls where needed:

```python
# Existing teleoperation loop
while teleoperation_active:
    gello_state = gello.get_joint_state()
    robot.command_joint_state(gello_state)
    
    # NEW: Add position locking when needed
    if should_lock_position:
        gello.lock_current_position()
        # Do other operations...
        gello.unlock_position()
```

This enhancement makes GELLO more versatile while maintaining the intuitive teleoperation experience. 