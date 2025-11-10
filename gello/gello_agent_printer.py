import time
import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

from gello.vader_teleop_config_reader import VADERTeleopConfigReader

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot

from serial.serialutil import SerialException

np.set_printoptions(suppress=True, precision=3)

# Set your serial port here

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

config_reader = VADERTeleopConfigReader()

PORT_TELEOP_G = config_reader.get_teleop_gripper_port()
PORT_TELEOP_C = config_reader.get_teleop_cutter_port()

robot_G_config = DynamixelRobotConfig(
    joint_ids=tuple(config_reader.get_teleop_gripper_ids()),
    joint_offsets=tuple(
        offset * (np.pi / 2) for offset in config_reader.get_teleop_gripper_offsets()
    ),
    joint_signs=tuple(config_reader.get_teleop_gripper_signs()),
    gripper_config=tuple(config_reader.get_teleop_gripper_config()),
)
try:    
    teleop_G_robot = robot_G_config.make_robot(port=PORT_TELEOP_G)
except SerialException as e:
    print(f"Failed to connect to teleop G robot on port {PORT_TELEOP_G}: {e}")
    teleop_G_robot = None
except RuntimeError as e:
    print(f"Runtime error while connecting to teleop G robot on port {PORT_TELEOP_G}: {e}")
    print("Are the teleop motors connected properly?")
    teleop_G_robot = None

robot_C_config = DynamixelRobotConfig(
    joint_ids=tuple(config_reader.get_teleop_cutter_ids()),
    joint_offsets=tuple(
        offset * (np.pi / 2) for offset in config_reader.get_teleop_cutter_offsets()
    ),
    joint_signs=tuple(config_reader.get_teleop_cutter_signs()),  # assuming all joints have positive sign
    gripper_config=tuple(config_reader.get_teleop_cutter_config()),
)
try:    
    teleop_C_robot = robot_C_config.make_robot(port=PORT_TELEOP_C)
except SerialException as e:
    print(f"Failed to connect to teleop C robot on port {PORT_TELEOP_C}: {e}")
    teleop_C_robot = None
except RuntimeError as e:
    print(f"Runtime error while connecting to teleop C robot on port {PORT_TELEOP_C}: {e}")
    print("Are the teleop motors connected properly?")
    teleop_C_robot = None


if teleop_G_robot is None and teleop_C_robot is None:
    print("No teleop consoles connected. Exiting.")
    exit(1)

# Print joint values continuously
try:
    while True:
        if teleop_G_robot:
            joints = teleop_G_robot.get_joint_state()  # Should return np.ndarray
            print("Teleop G Joint values (radians):", np.round(joints, 3))
        if teleop_C_robot:
            joints = teleop_C_robot.get_joint_state()  # Should return np.ndarray
            print("Teleop C Joint values (radians):", np.round(joints, 3))
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped.")
