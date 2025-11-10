import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot

from gello.vader_teleop_config_reader import VADERTeleopConfigReader
@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Tuple[int, int, int]
    """The gripper config of GELLO. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position)."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self, port: str, start_joints: Optional[np.ndarray] = None
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
        )

class GelloAgent(Agent):
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        # Ensure start_joints is a numpy array if provided
        if start_joints is not None and not isinstance(start_joints, np.ndarray):
            start_joints = np.array(start_joints)
        if dynamixel_config is not None:
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
        else:
            config_reader = VADERTeleopConfigReader()

            PORT_TELEOP_G = config_reader.get_teleop_gripper_port()
            PORT_TELEOP_C = config_reader.get_teleop_cutter_port()

            if port == PORT_TELEOP_G:
                robot_G_config = DynamixelRobotConfig(
                    joint_ids=tuple(config_reader.get_teleop_gripper_ids()),
                    joint_offsets=tuple(
                        offset * (np.pi / 2) for offset in config_reader.get_teleop_gripper_offsets()
                    ),
                    joint_signs=tuple(config_reader.get_teleop_gripper_signs()),  # assuming all joints have positive sign
                    gripper_config=tuple(config_reader.get_teleop_gripper_config()),
                )
                self._robot = robot_G_config.make_robot(port=port, start_joints=start_joints)

            elif port == PORT_TELEOP_C:
                robot_C_config = DynamixelRobotConfig(
                    joint_ids=tuple(config_reader.get_teleop_cutter_ids()),
                    joint_offsets=tuple(
                        offset * (np.pi / 2) for offset in config_reader.get_teleop_cutter_offsets()
                    ),
                    joint_signs=tuple(config_reader.get_teleop_cutter_signs()),  # assuming all joints have positive sign
                    gripper_config=tuple(config_reader.get_teleop_cutter_config()),
                )
                self._robot = robot_C_config.make_robot(port=port, start_joints=start_joints)
            else:
                raise ValueError(f"Port {port} does not match any known teleop ports.")

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        return self._robot.get_joint_state()
