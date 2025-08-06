"""
GELLO Dynamixel Robot Interface

This module implements the DynamixelRobot class, which provides a high-level interface
for controlling GELLO teleoperation devices built with Dynamixel servo motors.

GELLO (Generalized and Efficient Low-cost Low-latency Teleoperation) is a physical
device that mimics the joint structure of robotic arms, allowing humans to intuitively
control robots through direct manipulation. The device consists of multiple Dynamixel
servo motors arranged in the same kinematic configuration as the target robot.

Key Components:
- DynamixelRobot: Main class for GELLO device control
- Coordinate transformations: Convert between motor encoders and joint angles
- Position locking: Enable/disable motor torque for different operation modes
- Gripper support: Handle gripper motors with position mapping
- Teleoperation interface: Provide joint states for robot control

Usage:
    # Create GELLO device from configuration
    from gello.agents.gello_agent import PORT_CONFIG_MAP
    config = PORT_CONFIG_MAP["/dev/serial/by-id/your-device-id"]
    gello = config.make_robot(port="/dev/serial/by-id/your-device-id")
    
    # Read joint positions for teleoperation
    joint_positions = gello.get_joint_state()
    
    # Lock/unlock motors
    gello.lock_current_position()  # Hold current pose
    gello.unlock_position()        # Allow free movement

The class handles all the low-level details of Dynamixel communication, coordinate
transformations, and provides a clean interface that matches the Robot protocol
used throughout the GELLO system.
"""

from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.robots.robot import Robot


class DynamixelRobot(Robot):
    """A class representing a GELLO teleoperation device built with Dynamixel motors.
    
    This class provides a high-level interface for controlling a physical GELLO device,
    which is used as an input controller for robot teleoperation. The GELLO device
    consists of multiple Dynamixel servo motors arranged to mimic the joint structure
    of target robots.
    
    Key features:
    - Joint position reading for teleoperation input
    - Motor torque control (lock/unlock positions)  
    - Coordinate transformations (joint offsets and signs)
    - Gripper support for manipulation tasks
    - Exponential smoothing for stable position readings
    
    The class handles the conversion between raw motor encoder values and 
    meaningful joint angles in the robot's coordinate system.
    """

    def __init__(
        self,
        joint_ids: Sequence[int],
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        real: bool = False,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        from gello.dynamixel.driver import (
            DynamixelDriver,
            DynamixelDriverProtocol,
            FakeDynamixelDriver,
        )

        print(f"attempting to connect to port: {port}")
        self.gripper_open_close: Optional[Tuple[float, float]]
        if gripper_config is not None:
            assert joint_offsets is not None
            assert joint_signs is not None

            # joint_ids.append(gripper_config[0])
            # joint_offsets.append(0.0)
            # joint_signs.append(1)
            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1] * np.pi / 180,
                gripper_config[2] * np.pi / 180,
            )
        else:
            self.gripper_open_close = None

        self._joint_ids = joint_ids
        self._driver: DynamixelDriverProtocol

        if joint_offsets is None:
            self._joint_offsets = np.zeros(len(joint_ids))
        else:
            self._joint_offsets = np.array(joint_offsets)

        if joint_signs is None:
            self._joint_signs = np.ones(len(joint_ids))
        else:
            self._joint_signs = np.array(joint_signs)

        assert len(self._joint_ids) == len(self._joint_offsets), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_offsets: {len(self._joint_offsets)}"
        )
        assert len(self._joint_ids) == len(self._joint_signs), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_signs: {len(self._joint_signs)}"
        )
        assert np.all(
            np.abs(self._joint_signs) == 1
        ), f"joint_signs: {self._joint_signs}"

        if real:
            self._driver = DynamixelDriver(joint_ids, port=port, baudrate=baudrate)
            self._driver.set_torque_mode(False)
        else:
            self._driver = FakeDynamixelDriver(joint_ids)
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99

        if start_joints is not None:
            # loop through all joints and add +- 2pi to the joint offsets to get the closest to start joints
            new_joint_offsets = []
            current_joints = self.get_joint_state()
            assert current_joints.shape == start_joints.shape
            if gripper_config is not None:
                current_joints = current_joints[:-1]
                start_joints = start_joints[:-1]
            for idx, (c_joint, s_joint, joint_offset) in enumerate(
                zip(current_joints, start_joints, self._joint_offsets)
            ):
                new_joint_offsets.append(
                    np.pi
                    * 2
                    * np.round((-s_joint + c_joint) / (2 * np.pi))
                    * self._joint_signs[idx]
                    + joint_offset
                )
            if gripper_config is not None:
                new_joint_offsets.append(self._joint_offsets[-1])
            self._joint_offsets = np.array(new_joint_offsets)

    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()

        if self.gripper_open_close is not None:
            # map pos to [0, 1]
            g_pos = (pos[-1] - self.gripper_open_close[0]) / (
                self.gripper_open_close[1] - self.gripper_open_close[0]
            )
            g_pos = min(max(0, g_pos), 1)
            pos[-1] = g_pos

        if self._last_pos is None:
            self._last_pos = pos
        else:
            # exponential smoothing
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        self._driver.set_joints((joint_state + self._joint_offsets).tolist())

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def lock_current_position(self):
        """Lock GELLO at its current position by enabling torque.
        
        This powers the motors and holds them at their current joint angles,
        preventing manual movement. Useful for:
        - Holding a specific pose during robot operation
        - Preventing accidental movement during calibration
        - Creating a stable reference position
        """
        self._driver.lock_current_position()
        self._torque_on = True

    def unlock_position(self):
        """Unlock GELLO by disabling torque, allowing manual movement.
        
        This is the normal teleoperation mode where you can freely move
        the GELLO device and it will track the movements.
        """
        self._driver.unlock_position()
        self._torque_on = False

    def lock_at_joint_angles(self, joint_angles: np.ndarray):
        """Lock GELLO at specific joint angles.
        
        Args:
            joint_angles: Target joint angles in the robot's coordinate system
        """
        if len(joint_angles) != len(self._joint_ids):
            if self.gripper_open_close is not None and len(joint_angles) == len(self._joint_ids) - 1:
                # Add gripper position if not provided
                current_state = self.get_joint_state()
                joint_angles = np.append(joint_angles, current_state[-1])
            else:
                raise ValueError(f"Expected {len(self._joint_ids)} joint angles, got {len(joint_angles)}")
        
        # Convert to raw motor positions
        raw_positions = (joint_angles * self._joint_signs + self._joint_offsets).tolist()
        
        self._driver.lock_at_position(raw_positions)
        self._torque_on = True
        print(f"🤖 GELLO locked at target joint angles: {joint_angles}")

    def is_locked(self) -> bool:
        """Check if GELLO is currently locked (motors powered)."""
        return self._driver.is_locked()

    def get_lock_status(self) -> str:
        """Get human-readable lock status."""
        if self.is_locked():
            return "🔒 LOCKED (motors powered, position held)"
        else:
            return "🔓 UNLOCKED (free movement, ready for teleoperation)"

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}
