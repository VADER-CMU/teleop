import time
import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple


from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot

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
        self, port: str = "/dev/ttyUSB2", start_joints: Optional[np.ndarray] = None
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


PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # xArm
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U2O3-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=(
            0 * np.pi / 2,
            3 * np.pi / 2,
            3 * np.pi / 2,
            1 * np.pi / 2,
            -1 * np.pi / 2,
            4 * np.pi / 2,
            2 * np.pi / 2,
        ),
        joint_signs=(1, 1, 1, 1, 1, 1, 1),
        gripper_config=(8, 192, 150),
    ),
    # yam
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U4GA-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=[
            0 * np.pi,
            2 * np.pi / 2,
            4 * np.pi / 2,
            6 * np.pi / 6,
            5 * np.pi / 3,
            2 * np.pi / 2,
        ],
        joint_signs=(1, -1, -1, -1, 1, 1),
        gripper_config=(
            7,
            -30,
            24,
        ),  # Reversed: now starts open (-30) and closes on press (24)
    ),
    # Left UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            0,
            1 * np.pi / 2 + np.pi,
            np.pi / 2 + 0 * np.pi,
            0 * np.pi + np.pi / 2,
            np.pi - 2 * np.pi / 2,
            -1 * np.pi / 2 + 2 * np.pi,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 20, -22),
    ),
    # Right UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            np.pi + 0 * np.pi,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            1 * np.pi,
            3 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 286, 248),
    ),
}


PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U2O3-if00-port0"  # example

if PORT not in PORT_CONFIG_MAP:
    raise ValueError(f"Port {PORT} not in configuration map.")

# Initialize robot using the config
robot_config = PORT_CONFIG_MAP[PORT]
robot = robot_config.make_robot(port=PORT)

# Print joint values continuously
try:
    while True:
        joints = robot.get_joint_state()  # Should return np.ndarray
        print("Joint values (radians):", np.round(joints, 3))
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped.")
