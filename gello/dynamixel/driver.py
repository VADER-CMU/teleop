import os
import subprocess
import time
from threading import Event, Lock, Thread
from typing import Optional, Protocol, Sequence, Tuple

import numpy as np
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
)

import threading

# Constants
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
# Additional control table addresses and lengths for current mode and velocities
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4
ADDR_OPERATING_MODE = 11
CURRENT_CONTROL_MODE = 0
POSITION_CONTROL_MODE = 3
EXTENDED_POSITION_CONTROL_MODE = 4

# Servo-specific mappings and limits
TORQUE_TO_CURRENT_MAPPING = {
    "XC330_T288_T": 1158.73,
    "XM430_W210_T": 1000 / 2.69,
}

# Servo specifications for current limits (in mA)
SERVO_CURRENT_LIMITS = {
    "XC330_T288_T": 1193,
    "XM430_W210_T": 1263,
}


class DynamixelDriverProtocol(Protocol):
    def set_joints(self, joint_angles: Sequence[float]):
        """Set the joint angles for the Dynamixel servos.

        Args:
            joint_angles (Sequence[float]): A list of joint angles.
        """
        ...

    def set_current(self, currents: Sequence[float]):
        """Set motor currents (mA) for current control mode."""
        ...

    def set_torque(self, torques: Sequence[float]):
        """Set joint torques (Nm), mapped to motor currents using servo mappings."""
        ...

    def set_operating_mode(self, mode: int):
        """Set the operating mode (e.g., CURRENT_CONTROL_MODE or POSITION_CONTROL_MODE)."""
        ...

    def verify_operating_mode(self, expected_mode: int):
        """Verify that servos are in the expected operating mode."""
        ...

    def torque_enabled(self) -> bool:
        """Check if torque is enabled for the Dynamixel servos.

        Returns:
            bool: True if torque is enabled, False if it is disabled.
        """
        ...

    def set_torque_mode(self, enable: bool):
        """Set the torque mode for the Dynamixel servos.

        Args:
            enable (bool): True to enable torque, False to disable.
        """
        ...

    def get_joints(self) -> np.ndarray:
        """Get the current joint angles in radians.

        Returns:
            np.ndarray: An array of joint angles.
        """
        ...

    def get_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get joint positions (rad) and velocities (rad/s)."""
        ...

    def close(self):
        """Close the driver."""


class FakeDynamixelDriver(DynamixelDriverProtocol):
    def __init__(self, ids: Sequence[int]):
        self._ids = ids
        self._joint_angles = np.zeros(len(ids), dtype=float)
        self._velocities = np.zeros(len(ids), dtype=float)
        self._currents = np.zeros(len(ids), dtype=float)
        self._torque_enabled = False

    def set_joints(self, joint_angles: Sequence[float]):
        if len(joint_angles) != len(self._ids):
            raise ValueError(
                "The length of joint_angles must match the number of servos"
            )
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set joint angles")
        self._joint_angles = np.array(joint_angles, dtype=float)

    def set_current(self, currents: Sequence[float]):
        if len(currents) != len(self._ids):
            raise ValueError("The length of currents must match the number of servos")
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set currents")
        self._currents = np.array(currents, dtype=float)

    def set_torque(self, torques: Sequence[float]):
        # For fake driver, treat torques as currents for storage
        self.set_current(torques)

    def set_operating_mode(self, mode: int):
        self._operating_mode = mode

    def verify_operating_mode(self, expected_mode: int):
        pass

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        self._torque_enabled = enable

    def get_joints(self) -> np.ndarray:
        return self._joint_angles.copy()

    def get_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._joint_angles.copy(), self._velocities.copy()

    def get_positions(self) -> np.ndarray:
        return self.get_joints()

    def close(self):
        pass



class DynamixelDriver(DynamixelDriverProtocol):
    def __init__(
        self, ids: Sequence[int], port: str = "/dev/ttyUSB0", baudrate: int = 57600, operating_mode: int = POSITION_CONTROL_MODE
    ):
        """Initialize the DynamixelDriver class.

        Args:
            ids (Sequence[int]): A list of IDs for the Dynamixel servos.
            port (str): The USB port to connect to the arm.
            baudrate (int): The baudrate for communication.
        """
        self._ids = ids
        self._joint_angles = None
        self._operating_mode = operating_mode
        self._lock = Lock() 

        # Initialize the port handler, packet handler, and group sync read/write
        self._portHandler = PortHandler(port)
        self._packetHandler = PacketHandler(2.0)
        self._groupSyncRead = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        self._groupSyncWrite = GroupSyncWrite(
            self._portHandler,
            self._packetHandler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        # Open the port and set the baudrate
        if not self._portHandler.openPort():
            raise RuntimeError("Failed to open the port")

        if not self._portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to change the baudrate, {baudrate}")

        # Add parameters for each Dynamixel servo to the group sync read
        for dxl_id in self._ids:
            if not self._groupSyncRead.addParam(dxl_id):
                raise RuntimeError(
                    f"Failed to add parameter for Dynamixel with ID {dxl_id}"
                )

        # Disable torque for each Dynamixel servo
        self._torque_enabled = False

        # set operating mode
        self.set_operating_mode(self._operating_mode)

        try:
            self.set_torque_mode(self._torque_enabled)
        except Exception as e:
            print(f"port: {port}, {e}")
    
    def set_operating_mode(self, mode: int):
        with self._lock:
            # disable torque before changing operating mode
            if self._torque_enabled:
                self.set_torque_mode(False)
            for dxl_id in self._ids:
                dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_OPERATING_MODE, mode
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print("Result:", dxl_comm_result)
                    print("Error: ", dxl_error)
                    raise RuntimeError(
                        f"Failed to set operating mode for Dynamixel with ID {dxl_id}"
                    )
            self._operating_mode = mode

    def set_joints(self, joint_angles: Sequence[float]):
        with self._lock:
            if len(joint_angles) != len(self._ids):
                raise ValueError(
                    "The length of joint_angles must match the number of servos"
                )
            if not self._torque_enabled:
                raise RuntimeError("Torque must be enabled to set joint angles")

            for dxl_id, angle in zip(self._ids, joint_angles):
                # Convert the angle to the appropriate value for the servo
                position_value = int(angle * 2048 / np.pi)
                mode = self._operating_mode
                if mode == EXTENDED_POSITION_CONTROL_MODE:
                    param_goal_position = position_value.to_bytes(4, byteorder='little', signed=True)
                else:
                    # Allocate goal position value into byte array
                    param_goal_position = position_value.to_bytes(4, byteorder='little')
                # param_goal_position = [
                #     DXL_LOBYTE(DXL_LOWORD(position_value)),
                #     DXL_HIBYTE(DXL_LOWORD(position_value)),
                #     DXL_LOBYTE(DXL_HIWORD(position_value)),
                #     DXL_HIBYTE(DXL_HIWORD(position_value)),
                # ]

                # Add goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self._groupSyncWrite.addParam(
                    dxl_id, param_goal_position
                )
                if not dxl_addparam_result:
                    raise RuntimeError(
                        f"Failed to set joint angle for Dynamixel with ID {dxl_id}"
                    )

            # Syncwrite goal position
            dxl_comm_result = self._groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Failed to syncwrite goal position: {dxl_comm_result}")
                # raise RuntimeError("Failed to syncwrite goal position")
            # else:
                # print("Success")
            # Clear syncwrite parameter storage
            self._groupSyncWrite.clearParam()

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        with self._lock:
            torque_value = TORQUE_ENABLE if enable else TORQUE_DISABLE
            for dxl_id in self._ids:
                dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_TORQUE_ENABLE, torque_value
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print("Result:", dxl_comm_result)
                    print("Error: ", dxl_error)
                    raise RuntimeError(
                        f"Failed to set torque mode for Dynamixel with ID {dxl_id}"
                    )

            self._torque_enabled = enable

    def _read_joint_angles(self):
        with self._lock:
            # Continuously read joint angles and update the joint_angles array
            _joint_angles = np.zeros(len(self._ids), dtype=int)
            dxl_comm_result = self._groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(f"warning, comm failed: {dxl_comm_result}")
            for i, dxl_id in enumerate(self._ids):
                if self._groupSyncRead.isAvailable(
                    dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                ):
                    angle = self._groupSyncRead.getData(
                        dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    )
                    angle = np.int32(np.uint32(angle))
                    _joint_angles[i] = angle
                else:
                    raise RuntimeError(
                        f"Failed to get joint angles for Dynamixel with ID {dxl_id}"
                    )
            self._joint_angles = _joint_angles
            # self._groupSyncRead.clearParam() # TODO what does this do? should i add it

    def get_joints(self) -> np.ndarray:
        # Return a copy of the joint_angles array to avoid race conditions
        # with self._lock:
        self._read_joint_angles()
        _j = self._joint_angles.copy()
        return _j / 2048.0 * np.pi

    def close(self):
        with self._lock:
            self._torque_enabled = False
            try:
                self.set_torque_mode(self._torque_enabled)
            except Exception as e:
                print(f"port: , {e}")

            self._portHandler.closePort()

def main():
    # Set the port, baudrate, and servo IDs
    ids = [1]

    # Create a DynamixelDriver instance
    try:
        driver = DynamixelDriver(ids)
    except FileNotFoundError:
        driver = DynamixelDriver(ids, port="/dev/cu.usbserial-FT7WBMUB")

    # Test setting torque mode
    driver.set_torque_mode(True)
    driver.set_torque_mode(False)

    # Test reading the joint angles
    try:
        while True:
            joint_angles = driver.get_joints()
            print(f"Joint angles for IDs {ids}: {joint_angles}")
            # print(f"Joint angles for IDs {ids[1]}: {joint_angles[1]}")
    except KeyboardInterrupt:
        driver.close()


if __name__ == "__main__":
    main()  # Test the driver
