import dataclasses
import threading
import time
from typing import Dict, Optional

import numpy as np
from pyquaternion import Quaternion

from gello.robots.robot import Robot

from gello.dynamixel.driver import DynamixelDriver


def _aa_from_quat(quat: np.ndarray) -> np.ndarray:
    """Convert a quaternion to an axis-angle representation.

    Args:
        quat (np.ndarray): The quaternion to convert.

    Returns:
        np.ndarray: The axis-angle representation of the quaternion.
    """
    assert quat.shape == (4,), "Input quaternion must be a 4D vector."
    norm = np.linalg.norm(quat)
    assert norm != 0, "Input quaternion must not be a zero vector."
    quat = quat / norm  # Normalize the quaternion

    Q = Quaternion(w=quat[3], x=quat[0], y=quat[1], z=quat[2])
    angle = Q.angle
    axis = Q.axis
    aa = axis * angle
    return aa


def _quat_from_aa(aa: np.ndarray) -> np.ndarray:
    """Convert an axis-angle representation to a quaternion.

    Args:
        aa (np.ndarray): The axis-angle representation to convert.

    Returns:
        np.ndarray: The quaternion representation of the axis-angle.
    """
    assert aa.shape == (3,), "Input axis-angle must be a 3D vector."
    norm = np.linalg.norm(aa)
    assert norm != 0, "Input axis-angle must not be a zero vector."
    axis = aa / norm  # Normalize the axis-angle

    Q = Quaternion(axis=axis, angle=norm)
    quat = np.array([Q.x, Q.y, Q.z, Q.w])
    return quat


@dataclasses.dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    z: float
    gripper: float #changes comment out
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float
    j7: float
    aa: np.ndarray

    @staticmethod
    def from_robot(
        cartesian: np.ndarray,
        joints: np.ndarray,
        gripper: float,
        aa: np.ndarray,
    ) -> "RobotState":
        return RobotState(
            cartesian[0],
            cartesian[1],
            cartesian[2],
            gripper, #changes comment out
            joints[0],
            joints[1],
            joints[2],
            joints[3],
            joints[4],
            joints[5],
            joints[6],
            aa,
        )

    def cartesian_pos(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def quat(self) -> np.ndarray:
        return _quat_from_aa(self.aa)

    def joints(self) -> np.ndarray:
        return np.array([self.j1, self.j2, self.j3, self.j4, self.j5, self.j6, self.j7])

    def gripper_pos(self) -> float: #Changes comment out
        return self.gripper


class Rate:
    def __init__(self, *, duration):
        self.duration = duration
        self.last = time.time()

    def sleep(self, duration=None) -> None:
        duration = self.duration if duration is None else duration
        assert duration >= 0
        now = time.time()
        passed = now - self.last
        remaining = duration - passed
        assert passed >= 0
        if remaining > 0.0001:
            time.sleep(remaining)
        self.last = time.time()

class PepperGripper:
    def __init__(self, joint_ids, port_tool, baudrate: int = 57600):
        self.port_tool = port_tool
        # connect to dynamixel
        if self.port_tool is not None:
            self._driver = DynamixelDriver(joint_ids, port=port_tool, baudrate=baudrate)
            self._driver.set_torque_mode(True)
        self._torque_on = True
        # TODO get min max values here
        self.init_pos = self._driver.get_joints()
        # self.gripper_open = self.init_pos
        self.gripper_open = [30 * (np.pi/180), 30 * (np.pi/180), 30 * (np.pi/180)]
        # self.gripper_close = np.copy(self.init_pos)
        # self.gripper_close[0] += 0.8 * np.pi
        # self.gripper_close[1] += 0.8 * np.pi
        # self.gripper_close[2] += 0.8 * np.pi
        self.gripper_close = [270 * (np.pi/180), 270 * (np.pi/180), 270 * (np.pi/180)]

    def set_gripper_position(self, pos: float, wait: bool = False) -> None:
        if self.port_tool is None:
            return
        # clip pos to be between 0 and 1
        pos = np.clip(pos, 0.0, 1.0)
        #  pos is 0.0 for open gripper, 1.0 for closed gripper
        pos_cmd = np.array(self.gripper_open) + pos * (np.array(self.gripper_close) - np.array(self.gripper_open))
        # pos_cmd = [512, 512, 512]  # open position
        print(f"pos_cmd: {pos_cmd}")
        # self._driver.set_joints(pos_cmd)
        try:
            self._driver.set_joints(pos_cmd)
        except Exception as e:
            print(f"SyncWrite failed for motors with positions {pos_cmd}")
            print(f"Error details: {str(e)}")
            raise

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_joints(self):
        return self._driver.get_joints()

    def get_position(self):
        # return normalized position
        return (self._driver.get_joints()[0] - self.gripper_open[0]) / (self.gripper_close[0] - self.gripper_open[0])


class PepperCutter:
    def __init__(self, joint_ids, port_tool, baudrate: int = 57600):
        self.port_tool = port_tool
        # connect to dynamixel
        if self.port_tool is not None:
            self._driver = DynamixelDriver([joint_ids], port=port_tool, baudrate=baudrate)
            self._driver.set_torque_mode(True)
        self._torque_on = True
        # TODO get min max values here
        self.init_pos = self._driver.get_joints()
        self.gripper_open = np.array([205.05]) * np.pi / 180 # self.init_pos
        self.gripper_close = np.array([342.77]) * np.pi / 180 #np.copy(self.init_pos)
        self.gripper_close += 110.0 / 180.0 * np.pi
        # print("Peppercutter: gripper open pos:", self.gripper_open)
        # print("Peppercutter: gripper close pos:", self.gripper_close)

    def set_gripper_position(self, pos: float, wait: bool = False) -> None:
        if self.port_tool is None:
            return
        # clip pos to be between 0 and 1
        # print(f"Setting cutter position to {pos} clipped between 0 and 1")
        pos = np.clip(pos, 0.0, 1.0)
        #  pos is 0.0 for open gripper, 1.0 for closed gripper
        pos_cmd = np.array(self.gripper_open) + pos * (np.array(self.gripper_close) - np.array(self.gripper_open))
        self._driver.set_joints(pos_cmd)

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_joints(self):
        return self._driver.get_joints()

    def get_position(self):
        # return normalized position
        return (self._driver.get_joints()[0] - self.gripper_open[0]) / (self.gripper_close[0] - self.gripper_open[0])
    
class XArmRobotGripper(Robot):
    GRIPPER_OPEN = 0.0 
    GRIPPER_CLOSE = 1.0
    #  MAX_DELTA = 0.2
    DEFAULT_MAX_DELTA = 0.01 #0.05

    def num_dofs(self) -> int:
        return 8 

    def get_joint_state(self) -> np.ndarray:
        state = self.get_state()
        gripper = state.gripper_pos() 
        all_dofs = np.concatenate([state.joints(), np.array([gripper])])  
        return all_dofs

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        if len(joint_state) == 7:
            self.set_command(joint_state, None)
        elif len(joint_state) == 8:
            self.set_command(joint_state[:7], joint_state[7])
        else:
            raise ValueError(
                f"Invalid joint state: {joint_state}, len={len(joint_state)}"
            )

    def stop(self):
        self.running = False
        if self.robot is not None:
            self.robot.disconnect()

        if self.command_thread is not None:
            self.command_thread.join()


    def __init__(
        self,
        name: str = "xarm_right",
        ip: str = "192.168.1.226",
        real: bool = True,
        control_frequency: float = 100.0,#50.0,
        max_delta: float = DEFAULT_MAX_DELTA,
        port_tool: Optional[str] = None,
        ids_tool: Optional[list] = None,
        tool: int = 0,  # 0 for gripper, 1 for cutter
        ROS_control: bool = False,
    ):
        print(ip)
        self.real = real
        self.max_delta = max_delta
        if real:
            from xarm.wrapper import XArmAPI

            self.robot = XArmAPI(ip, is_radian=True)
            if tool == 0:
                self.gripper = PepperGripper(ids_tool, port_tool)
                self._set_gripper_position(self.GRIPPER_OPEN) #check if self.GRIPPER_OPEN is
            elif tool == 1:
                self.gripper = PepperCutter(ids_tool, port_tool)
                self._set_gripper_position(self.GRIPPER_OPEN)
        else:
            self.robot = None
        self._control_frequency = control_frequency
        


        if real and self.robot is not None:
            print(f"Initializing robot at {ip}...")
            self.robot.clean_error()
            time.sleep(0.5)
            self.robot.clean_warn()
            time.sleep(0.5)
            
            self.robot.motion_enable(True)
            time.sleep(0.5)
            
            self.robot.set_mode(1)  # Servo motion mode
            time.sleep(0.5)
            
            self.robot.set_state(state=0)  # Ready state
            time.sleep(1)  # Wait longer for ready state
            
            # Verify robot is ready before setting collision sensitivity
            code, state = self.robot.get_state()
            print(f"Robot state: code={code}, state={state}")
            
            if state == 0:
                # Set collision sensitivity only when ready
                ret = self.robot.set_collision_sensitivity(0)
                time.sleep(0.5)
                
                if ret == 0:
                    print(f"✓ Collision detection DISABLED for {ip}")
                else:
                    print(f"⚠ Failed to disable collision (code={ret}), trying level 5...")
                    ret = self.robot.set_collision_sensitivity(5)
                    if ret == 0:
                        print(f"✓ Collision sensitivity set to 5 (least sensitive) for {ip}")
                    else:
                        print(f"⚠ WARNING: Could not set collision sensitivity for {ip}")
            else:
                print(f"⚠ Cannot set collision sensitivity - robot state is {state}")



        self.last_state_lock = threading.Lock()
        self.target_command_lock = threading.Lock()
        self.last_state = self._update_last_state()
        self.target_command = {
            "joints": self.last_state.joints(),
            "gripper": 0.0,
        }
        self.running = True
        self.command_thread = None
        if real:
            self.command_thread = threading.Thread(target=self._robot_thread)
            self.command_thread.start()

    def get_state(self) -> RobotState:
        with self.last_state_lock:
            return self.last_state

    def set_command(self, joints: np.ndarray, gripper: Optional[float] = None) -> None:
        with self.target_command_lock:
            self.target_command = {
                "joints": joints,
                "gripper": gripper, #changes comment out
            }

    def _clear_error_states(self):
        if self.robot is None:
            return
        
        self.robot.clean_error()
        time.sleep(0.5)
        
        self.robot.clean_warn()
        time.sleep(0.3)
        
        self.robot.motion_enable(True)
        time.sleep(0.5)
        
        self.robot.set_mode(1)
        time.sleep(0.3)
        
        self.robot.set_state(state=0)
        time.sleep(0.5)
        # self.robot.set_gripper_enable(True) #commented out for now to avoid issues with gripper
        # time.sleep(1)
        # self.robot.set_gripper_mode(0)
        # time.sleep(1)
        # self.robot.set_gripper_speed(3000)
        # time.sleep(1)

    def _get_gripper_pos(self) -> float:
        if self.robot is None:
            return 0.0
        return self.gripper.get_position()
        # if self.robot is None: # commented out for now to avoid issues with gripper
        #     return 0.0
        # code, gripper_pos = self.robot.get_gripper_position()
        # while code != 0 or gripper_pos is None:
        #     print(f"Error code {code} in get_gripper_position(). {gripper_pos}")
        #     time.sleep(0.001)
        #     code, gripper_pos = self.robot.get_gripper_position()
        #     if code == 22:
        #         self._clear_error_states()

        # normalized_gripper_pos = (gripper_pos - self.GRIPPER_OPEN) / (
        #     self.GRIPPER_CLOSE - self.GRIPPER_OPEN
        # )
        # return normalized_gripper_pos

    def _set_gripper_position(self, pos: float) -> None:
        if self.gripper is None:
            return
        self.gripper.set_gripper_position(pos, wait=False)
        # while self.robot.get_is_moving():
        #     time.sleep(0.01)

    def _robot_thread(self):
        rate = Rate(
            duration=1 / self._control_frequency
        )  # command and update rate for robot
        step_times = []
        count = 0

        while self.running:
            s_t = time.time()
            # update last state
            self.last_state = self._update_last_state()
            with self.target_command_lock:
                joint_delta = np.array(
                    self.target_command["joints"] - self.last_state.joints()
                )
                gripper_command = self.target_command["gripper"]
                # gripper_command = self.target_command["gripper"] # commented out for now to avoid issues with gripper

            norm = np.linalg.norm(joint_delta)

            # threshold delta to be at most 0.01 in norm space
            if norm > self.max_delta:
                delta = joint_delta / norm * self.max_delta
            else:
                delta = joint_delta

            # command position
            self._set_position(
                self.last_state.joints() + delta,
            )

            if gripper_command is not None: 
                set_point = gripper_command
                self._set_gripper_position(
                    self.GRIPPER_OPEN
                    + set_point * (self.GRIPPER_CLOSE - self.GRIPPER_OPEN)
                )
            self.last_state = self._update_last_state()

            rate.sleep()
            step_times.append(time.time() - s_t)
            count += 1
            if count % 1000 == 0:
                # Mean, Std, Min, Max, only show 3 decimal places and string pad with 10 spaces
                frequency = 1 / np.mean(step_times)
                # print(f"Step time - mean: {np.mean(step_times):10.3f}, std: {np.std(step_times):10.3f}, min: {np.min(step_times):10.3f}, max: {np.max(step_times):10.3f}")
                print(
                    f"Low  Level Frequency - mean: {frequency:10.3f}, std: {np.std(frequency):10.3f}, min: {np.min(frequency):10.3f}, max: {np.max(frequency):10.3f}"
                )
                #Low  Level Frequency - mean:     24.311, std:      0.000, min:     24.311, max:     24.311

                step_times = []

    def _update_last_state(self) -> RobotState:
        with self.last_state_lock:
            if self.robot is None:
                return RobotState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, np.zeros(3))

            gripper_pos = self._get_gripper_pos() 

            code, servo_angle = self.robot.get_servo_angle(is_radian=True)
            while code != 0:
                print(f"Error code {code} in get_servo_angle().")
                self._clear_error_states()
                code, servo_angle = self.robot.get_servo_angle(is_radian=True)

            code, cart_pos = self.robot.get_position_aa(is_radian=True)
            while code != 0:
                print(f"Error code {code} in get_position().")
                self._clear_error_states()
                code, cart_pos = self.robot.get_position_aa(is_radian=True)

            cart_pos = np.array(cart_pos)
            aa = cart_pos[3:]
            cart_pos[:3] /= 1000

            return RobotState.from_robot(
                cart_pos,
                servo_angle,
                gripper_pos, #changes comment out
                aa,
            )

    def _set_position(
        self,
        joints: np.ndarray,
    ) -> None:
        if self.robot is None:
            return
        # threhold xyz to be in  min max

        # ret = self.robot.set_servo_angle_j(joints, wait=False, is_radian=True)
        # if ret in [1, 9]:
        #     self._clear_error_states()

        max_retries = 2
        for attempt in range(max_retries):
            ret = self.robot.set_servo_angle_j(joints, wait=False, is_radian=True)
            
            if ret == 0:  # Success
                return
            
            # Error occurred
            if ret in [1, 9]:
                if attempt < max_retries - 1:
                    print(f"Command error (code={ret}), retrying...")
                    self._clear_error_states()
                    time.sleep(0.2)
                else:
                    print(f"Command failed after {max_retries} attempts")
                    self._clear_error_states()
        


    def get_observations(self) -> Dict[str, np.ndarray]:
        state = self.get_state()
        pos_quat = np.concatenate([state.cartesian_pos(), state.quat()])
        joints = self.get_joint_state()
        gripper_position = self.gripper.get_joints()
        return {
            "joint_positions": joints,  # rotational joint + gripper state
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_position,
        }


def main():
    ip = "192.168.1.226"
    robot = XArmRobotGripper(
        ip=ip,
        port_tool="/dev/ttyUSB0",  # Your Dynamixel USB port
        ids_tool=[1, 2, 4],  # Your Dynamixel motor IDs
        tool=0  # 0 for gripper, 1 for cutter
    )
    import time

    time.sleep(1)
    print(robot.get_state())

    time.sleep(1)
    print(robot.get_state())
    print("end")
    robot.stop()


if __name__ == "__main__":
    main()
