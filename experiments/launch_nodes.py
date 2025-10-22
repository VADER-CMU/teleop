from dataclasses import dataclass
from pathlib import Path

import tyro

from gello.robots.robot import BimanualRobot, PrintRobot
from gello.zmq_core.robot_node import ZMQServerRobot

from typing import Sequence


@dataclass
class Args:
    robot: str = "xarm"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_right_ip: str = "192.168.1.195"  # right arm, change as required
    robot_left_ip: str = "192.168.1.230"  # left arm, change as required
    port_gripper: str = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89I74E-if00-port0" #change as required
    ids_gripper: Sequence[int] = (1, 2, 4)
    port_cutter: str = "/dev/ttyUSB4" #change as required
    ids_cutter: int = 0 
    ros: bool = False


def launch_robot_server(args: Args):
    port = args.robot_port
    if args.robot == "sim_ur":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_yam":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "i2rt_yam" / "yam.xml"
        from gello.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=None, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_panda":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "franka_emika_panda" / "panda.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_xarm":
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
            print("asdf")
            robot = XArmRobotGripper(name="xarm_right", ip=args.robot_right_ip, port_tool=args.port_gripper, ids_tool=args.ids_gripper, tool=0, ROS_control=False)

        #added bimanual xarm option
        elif args.robot == "bimanual_xarm":
            from gello.robots.xarm_robot import XArmRobotGripper

            # IP for the bimanual robot setup is hardcoded
            # _robot_l = XArmRobotGripper(ip="192.168.2.10")
            # _robot_r = XArmRobotGripper(ip="192.168.1.10")
            # robot = BimanualRobot(_robot_l, _robot_r)
            print("init gripper at " + args.port_gripper)
            print("gripper ids " + str(args.ids_gripper))

            print("init cutter at " + args.port_cutter)
            print("gripper ids " + str(args.ids_cutter))
            # if args.ros:
            #     import rospy
            #     rospy.init_node("robot_xarm")

            robot_right = XArmRobotGripper(name="xarm_right", ip=args.robot_right_ip, port_tool=args.port_gripper, ids_tool=args.ids_gripper, tool=0, ROS_control=False)
            robot_left = XArmRobotGripper(name="xarm_left", ip=args.robot_left_ip, port_tool=args.port_cutter, ids_tool=args.ids_cutter, tool=1, ROS_control=False)
            robot = BimanualRobot(robot_left, robot_right)


        elif args.robot == "ur":
            from gello.robots.ur import URRobot

            robot = URRobot(robot_ip=args.robot_right_ip)
        elif args.robot == "panda":
            from gello.robots.panda import PandaRobot

            robot = PandaRobot(robot_ip=args.robot_right_ip)
        elif args.robot == "bimanual_ur":
            from gello.robots.ur import URRobot

            # IP for the bimanual robot setup is hardcoded
            _robot_l = URRobot(robot_ip="192.168.2.10")
            _robot_r = URRobot(robot_ip="192.168.1.10")
            robot = BimanualRobot(_robot_l, _robot_r)
        elif args.robot == "yam":
            from gello.robots.yam import YAMRobot

            robot = YAMRobot(channel="can0")
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
    launch_robot_server(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
