"""
Simulation wrapper for xArm5 that uses xArm7 MuJoCo model with joint mapping.
"""
import numpy as np
from typing import Dict

from gello.robots.sim_robot import MujocoRobotServer


class MujocoXArm5Server(MujocoRobotServer):
    """
    A wrapper around MujocoRobotServer that maps xArm5 joint commands (6 DOF)
    to xArm7 simulation model (8 DOF) by keeping unused joints at fixed positions.
    
    xArm5 uses joints 1,2,4,6,7 from xArm7 (indices 0,1,3,5,6).
    Joints 3 and 5 (indices 2,4) are kept at fixed positions.
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # Override num_joints to report 6 DOF for xArm5 (5 joints + gripper)
        self._xarm5_num_joints = 6
        
        # Fixed positions for unused joints 3 and 5 (indices 2, 4)
        self._fixed_joint_positions = {
            2: 0.0,  # Joint 3 fixed at 0
            4: 0.0,  # Joint 5 fixed at 0
        }
        
        # Initialize joint command with fixed positions
        self._xarm7_joint_cmd = np.zeros(self._num_joints)  # Full 8 DOF
        for idx, pos in self._fixed_joint_positions.items():
            self._xarm7_joint_cmd[idx] = pos
    
    def num_dofs(self) -> int:
        """Return 6 DOF for xArm5 (5 joints + gripper)"""
        return self._xarm5_num_joints
    
    def get_joint_state(self) -> np.ndarray:
        """Return only the 5 active joints + gripper for xArm5"""
        full_state = super().get_joint_state()
        
        # Map from xArm7 (8 DOF) to xArm5 (6 DOF)
        # Extract joints 1,2,4,6,7 + gripper (indices 0,1,3,5,6,7)
        xarm5_state = np.array([
            full_state[0],  # joint 1
            full_state[1],  # joint 2
            full_state[3],  # joint 4
            full_state[5],  # joint 6
            full_state[6],  # joint 7
            full_state[7],  # gripper
        ])
        
        return xarm5_state
    
    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Map xArm5 joint commands (6 DOF) to xArm7 format (8 DOF)"""
        assert len(joint_state) == self._xarm5_num_joints, (
            f"Expected joint state of length {self._xarm5_num_joints}, "
            f"got {len(joint_state)}."
        )
        
        # Map xArm5 (6 DOF) to xArm7 (8 DOF)
        # xArm5 joints -> xArm7 indices: [0,1,3,5,6,7]
        self._xarm7_joint_cmd[0] = joint_state[0]  # joint 1
        self._xarm7_joint_cmd[1] = joint_state[1]  # joint 2
        # joint 3 (index 2) stays at fixed position
        self._xarm7_joint_cmd[3] = joint_state[2]  # joint 4
        # joint 5 (index 4) stays at fixed position  
        self._xarm7_joint_cmd[5] = joint_state[3]  # joint 6
        self._xarm7_joint_cmd[6] = joint_state[4]  # joint 7
        self._xarm7_joint_cmd[7] = joint_state[5]  # gripper
        
        # Handle gripper scaling if needed
        if self._has_gripper:
            _joint_cmd = self._xarm7_joint_cmd.copy()
            _joint_cmd[-1] = _joint_cmd[-1] * 255
            self._joint_cmd = _joint_cmd
        else:
            self._joint_cmd = self._xarm7_joint_cmd.copy()
    
    def get_observations(self) -> Dict[str, np.ndarray]:
        """Return observations mapped to xArm5 format"""
        obs = super().get_observations()
        
        # Map joint positions from xArm7 to xArm5
        full_joint_positions = obs["joint_positions"]
        xarm5_joint_positions = np.array([
            full_joint_positions[0],  # joint 1
            full_joint_positions[1],  # joint 2
            full_joint_positions[3],  # joint 4
            full_joint_positions[5],  # joint 6
            full_joint_positions[6],  # joint 7
        ])
        
        # Map joint velocities from xArm7 to xArm5
        full_joint_velocities = obs["joint_velocities"]
        xarm5_joint_velocities = np.array([
            full_joint_velocities[0],  # joint 1
            full_joint_velocities[1],  # joint 2
            full_joint_velocities[3],  # joint 4
            full_joint_velocities[5],  # joint 6
            full_joint_velocities[6],  # joint 7
        ])
        
        return {
            "joint_positions": xarm5_joint_positions,
            "joint_velocities": xarm5_joint_velocities,
            "ee_pos_quat": obs["ee_pos_quat"],
            "gripper_position": obs["gripper_position"],
        }
