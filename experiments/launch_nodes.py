import signal
import sys
from dataclasses import dataclass
from pathlib import Path

import tyro

from gello.robots.robot import BimanualRobot, PrintRobot
from gello.zmq_core.robot_node import ZMQServerRobot


@dataclass
class Args:
    robot: str = "xarm"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.1.10"


def launch_robot_server(args: Args):
    port = args.robot_port
    MENAGERIE_ROOT: Path = (
        Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
    )
    
    if args.robot == "sim_ur":
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=str(xml), gripper_xml_path=str(gripper_xml), port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_yam":
        xml = MENAGERIE_ROOT / "i2rt_yam" / "yam.xml"
        from gello.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=str(xml), gripper_xml_path=None, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_panda":
        from gello.robots.sim_robot import MujocoRobotServer

        xml = MENAGERIE_ROOT / "franka_emika_panda" / "panda.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=str(xml), gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_xarm":
        from gello.robots.sim_robot import MujocoRobotServer

        xml = MENAGERIE_ROOT / "ufactory_xarm7" / "xarm7.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=str(xml), gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_xarm5":
        from gello.robots.sim_xarm5_robot import MujocoXArm5Server

        # Use xArm7 model but with joint mapping for xArm5 (joints 1,2,4,6,7)
        xml = MENAGERIE_ROOT / "ufactory_xarm7" / "xarm7.xml"
        gripper_xml = None
        server = MujocoXArm5Server(
            xml_path=str(xml), gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    else:
        if args.robot == "xarm":
            from gello.robots.xarm_robot import XArmRobot

            robot = XArmRobot(ip=args.robot_ip)
        elif args.robot == "xarm5":
            from gello.robots.xarm5_robot import XArm5Robot

            robot = XArm5Robot(ip=args.robot_ip)
        elif args.robot == "ur":
            from gello.robots.ur import URRobot

            robot = URRobot(robot_ip=args.robot_ip)
        elif args.robot == "panda":
            from gello.robots.panda import PandaRobot

            robot = PandaRobot(robot_ip=args.robot_ip)
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
                f"Robot {args.robot} not implemented, choose one of: sim_ur, sim_xarm, sim_xarm5, sim_panda, sim_yam, xarm, xarm5, ur, bimanual_ur, yam, none"
            )
        server = ZMQServerRobot(robot, port=port, host=args.hostname)
        print(f"Starting robot server on port {port}")
        server.serve()


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nReceived interrupt signal. Shutting down...")
    sys.exit(0)


def main(args):
    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    print(f"Starting robot server for: {args.robot}")
    print("Press Ctrl+C to stop the server")
    
    try:
        launch_robot_server(args)
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Server stopped")


if __name__ == "__main__":
    main(tyro.cli(Args))
