from dataclasses import dataclass
from pathlib import Path

import tyro

from gello.robots.robot import BimanualRobot, PrintRobot
from gello.zmq_core.robot_node import ZMQServerRobot

from typing import Sequence

from gello.vader_teleop_config_reader import VADERTeleopConfigReader

config_reader = VADERTeleopConfigReader()


@dataclass
class Args:
    xarm_right_ip: str = config_reader.get_right_arm_ip()
    xarm_left_ip: str = config_reader.get_left_arm_ip()
    port_gripper: str = config_reader.get_gripper_port()
    ids_gripper: Sequence[int] = tuple(config_reader.get_gripper_ids())
    port_fhrsense: str = config_reader.get_fhrsense_port()
    ids_fhrsense: int = config_reader.get_fhrsense_ids()
    tool_ranges_fhrsense: Sequence[float] = tuple(
        config_reader.get_fhrsense_tool_ranges())
    robot: str = "xarm"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    ros: bool = False


def launch_robot_server(args: Args):
    port = args.robot_port
    if args.robot == "sim_xarm":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "ufactory_xarm7" / "xarm7.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    else:
        if args.robot == "xarm":
            from gello.robots.xarm_robot import XArmRobotGripper
            # robot = XArmRobotGripper(name="xarm_right", ip=args.xarm_cutter_ip, port_tool=args.port_cutter, ids_tool=args.ids_cutter, tool=1, ROS_control=False)
            robot = XArmRobotGripper(name="xarm_right", ip=args.xarm_gripper_ip,
                                     port_tool=args.port_gripper, ids_tool=args.ids_gripper, tool=0, ROS_control=False)

        # added bimanual xarm option
        elif args.robot == "bimanual_xarm":
            from gello.robots.xarm_robot import XArmRobotGripper

            print("init gripper at " + args.port_gripper)
            print("gripper ids " + str(args.ids_gripper))

            print("init fhrsense at " + args.port_fhrsense)
            print("fhrsense ids " + str(args.ids_fhrsense))
            # if args.ros:
            #     import rospy
            #     rospy.init_node("robot_xarm")

            robot_right = XArmRobotGripper(name="xarm_right", ip=args.xarm_right_ip,
                                           port_tool=args.port_fhrsense, ids_tool=args.ids_fhrsense, tool=1, ROS_control=False)
            print("initialized 1 arm")
            robot_left = XArmRobotGripper(name="xarm_left", ip=args.xarm_left_ip,
                                          port_tool=args.port_gripper, ids_tool=args.ids_gripper, tool=0, ROS_control=False)
            print("initialized both arms")
            robot = BimanualRobot(robot_left, robot_right)
        elif args.robot == "none" or args.robot == "print":
            robot = PrintRobot(8)

        else:
            raise NotImplementedError(
                f"Robot {args.robot} not implemented, choose one of: sim_ur, xarm, ur, bimanual_ur, none"
            )
        server = ZMQServerRobot(robot, port=port, host=args.hostname)
        print(f"Starting robot server on port {port}")
        server.serve()


def main(args):
    # config_reader = VADERTeleopConfigReader()
    # args.xarm_gripper_ip = config_reader.get_gripper_arm_ip()
    # args.xarm_cutter_ip = config_reader.get_cutter_arm_ip()
    # args.port_gripper = config_reader.get_gripper_port()
    # args.ids_gripper = config_reader.get_gripper_ids()
    # args.port_cutter = config_reader.get_cutter_port()
    # args.ids_cutter = config_reader.get_cutter_id()
    launch_robot_server(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
