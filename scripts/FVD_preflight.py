import sys
import platform
import subprocess

from gello.vader_teleop_config_reader import VADERTeleopConfigReader

from gello.dynamixel.driver import DynamixelDriver

import pyrealsense2 as rs
import time
#!/usr/bin/env python3

config_reader = VADERTeleopConfigReader()

def _ip_reachable(ip: str, timeout: float = 0.5) -> bool:
    
    cmd = ["ping", "-c", "1", ip]

    try:
        proc = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=timeout)
        return proc.returncode == 0
    except subprocess.TimeoutExpired:
        return False
    except Exception:
        return False
    
def _usb_device_exists(dev_full_path: str) -> bool:
    try:
        with open(dev_full_path, 'r'):
            return True
    except FileNotFoundError:
        return False

def gripper_arm_reachable() -> bool:
    gripper_ip = config_reader.get_gripper_arm_ip()
    return _ip_reachable(gripper_ip)

def fhrsense_arm_reachable() -> bool:
    fhrsense_ip = config_reader.get_fhrsense_arm_ip()
    return _ip_reachable(fhrsense_ip)

def gripper_u2d2_connected() -> bool:
    gripper_port = config_reader.get_gripper_port()
    return _usb_device_exists(gripper_port)

def fhrsense_u2d2_connected() -> bool:
    fhrsense_port = config_reader.get_fhrsense_port()
    return _usb_device_exists(fhrsense_port)

def teleop_g_u2d2_connected() -> bool:
    teleop_g_port = config_reader.get_teleop_gripper_port()
    return _usb_device_exists(teleop_g_port)

def teleop_c_u2d2_connected() -> bool:
    teleop_c_port = config_reader.get_teleop_fhrsense_port()
    return _usb_device_exists(teleop_c_port)

def _realsense_connected(expected_serial: str) -> bool:
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            if dev.get_info(rs.camera_info.serial_number) == expected_serial:
                return True
        print(f"⚠️ Realsense device with serial {expected_serial} not found.")
        print("Connected Realsense devices:", end="")
        for dev in devices:
            print(f" {dev.get_info(rs.camera_info.serial_number)}", end="")
        print()
        return False
    except Exception as e:
        print(f"⚠️ Error accessing Realsense devices: {e}")
        return False

def teleop_realsense_connected() -> bool:
    teleop_serial = config_reader.get_teleop_camera_serial()
    return _realsense_connected(teleop_serial)

def gripper_realsense_connected() -> bool:
    gripper_serial = config_reader.get_gripper_camera_serial()
    return _realsense_connected(gripper_serial)

def fhrsense_realsense_connected() -> bool:
    fhrsense_serial = config_reader.get_fhrsense_camera_serial()
    return _realsense_connected(fhrsense_serial)

def _xarm_operational(ip: str) -> bool:
    from xarm.wrapper import XArmAPI
    robot = XArmAPI(ip, is_radian=True)
    # Verify robot is ready before setting collision sensitivity
    code, state = robot.get_state()

    print(f"Robot state: code={code}, state={state}")

    robot.clean_error()
    time.sleep(0.5)
    
    robot.clean_warn()
    time.sleep(0.3)
    
    code, [error_code, warn_code] = robot.get_err_warn_code()

    print(f"Robot state: code={code}, err={error_code}, warn={warn_code}")

    if(error_code == 2):
        print("⚠️ Robot emergency stop engaged.")
        return False
    if(error_code != 0 or warn_code != 0): 
        print("⚠️ Robot has nonzero error state after clearing errors.")
        return False
    return True

def gripper_xarm_operational() -> bool:
    gripper_ip = config_reader.get_gripper_arm_ip()
    return _xarm_operational(gripper_ip)

def fhrsense_xarm_operational() -> bool:
    fhrsense_ip = config_reader.get_fhrsense_arm_ip()
    return _xarm_operational(fhrsense_ip)

def _u2d2_operational(port: str, ids: list, baudrate = 57600) -> bool:
    try:
        driver = DynamixelDriver(ids, port = port, baudrate = baudrate)
        driver.set_torque_mode(True)
        return True
    except Exception as e:
        print(f"⚠️ Error initializing DynamixelDriver on port {port}: {e}")
        return False

def gripper_u2d2_operational() -> bool:
    gripper_port = config_reader.get_gripper_port()
    gripper_ids = config_reader.get_gripper_ids()
    return _u2d2_operational(gripper_port, gripper_ids)

def fhrsense_u2d2_operational() -> bool:
    fhrsense_port = config_reader.get_fhrsense_port()
    fhrsense_ids = config_reader.get_fhrsense_ids()
    return _u2d2_operational(fhrsense_port, [fhrsense_ids])

def teleop_g_u2d2_operational() -> bool:
    teleop_g_port = config_reader.get_teleop_gripper_port()
    teleop_g_ids = config_reader.get_teleop_gripper_ids()
    return _u2d2_operational(teleop_g_port, teleop_g_ids)

def teleop_c_u2d2_operational() -> bool:
    teleop_c_port = config_reader.get_teleop_fhrsense_port()
    teleop_c_ids = config_reader.get_teleop_fhrsense_ids()
    return _u2d2_operational(teleop_c_port, teleop_c_ids)

def _test_realsense_capture(expected_serial: str) -> bool:
    """
    Checks if a RealSense device is connected and functional by attempting to start
    a pipeline and retrieve frames.
    """
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_device(str(expected_serial))

    try:
        profile = pipeline.start(config)
        streamer = profile.get_stream(rs.stream.color)
        for _ in range(2):
            frames = pipeline.wait_for_frames()
            if not frames:
                return False
        return True

    except RuntimeError as e:
        print(f"⚠️ RuntimeError: {e}")
        print("⚠️ RealSense device not found or could not be started.")
        return False
    except Exception as e:
        print(f"⚠️ An unexpected error occurred: {e}")
        return False
    finally:
        # Always stop the pipeline to release resources.
        try:
            pipeline.stop()
        except RuntimeError:
            # This can happen if start() failed and pipeline was never truly started.
            pass

def test_teleop_realsense_capture() -> bool:
    teleop_serial = config_reader.get_teleop_camera_serial()
    return _test_realsense_capture(teleop_serial)

def test_gripper_realsense_capture() -> bool:
    gripper_serial = config_reader.get_gripper_camera_serial()
    return _test_realsense_capture(gripper_serial)

def test_fhrsense_realsense_capture() -> bool:
    fhrsense_serial = config_reader.get_fhrsense_camera_serial()
    return _test_realsense_capture(fhrsense_serial)

if __name__ == "__main__":
    # Test gripper first, then fhrsense and print results

    print("=====  Starting VADER FVD Preflight Checklist  =====")

    print("=====Checking Arms Control Boxes: Link Layer   =====")

    ip_devices = {"Gripper": gripper_arm_reachable, "FHRSense": fhrsense_arm_reachable}

    for name, check_fn in ip_devices.items():
        reachable = check_fn()
        if reachable:
            print(f"✅ {name} arm reachable by ping")
        else:
            print(f"⚠️ {name} arm not reachable, check power and ethernet connections")

    print("=====Checking Dynamixel U2D2 Bridges Link Layer=====")

    devices = {"Gripper": gripper_u2d2_connected, "FHRSense": fhrsense_u2d2_connected,
               "Teleop Gripper": teleop_g_u2d2_connected, "Teleop FHRSense": teleop_c_u2d2_connected}

    for name, check_fn in devices.items():
        connected = check_fn()
        if connected:
            print(f"✅ {name} U2D2 connected from /dev")
        else:
            print(f"⚠️ {name} U2D2 not connected, check USB and power connection")

    print("=====Checking Realsense Cameras Link Layer     =====")

    devices = {"Teleop Cam": teleop_realsense_connected, "Gripper Cam": gripper_realsense_connected, "FHRSense Cam": fhrsense_realsense_connected}


    for name, check_fn in devices.items():
        connected = check_fn()
        if connected:
            print(f"✅ {name} Realsense camera listed by pyrealsense2")
        else:
            print(f"⚠️ {name} Realsense camera not connected, check USB and power connection")
    
    print()

    print("=====Checking Arms Control Boxes: Drivers      =====")

    xarm_devices = {"Gripper": gripper_xarm_operational, "FHRSense": fhrsense_xarm_operational}


    for name, check_fn in xarm_devices.items():
        reachable = check_fn()
        if reachable:
            print(f"✅ {name} arm has no unexpected errors/warns")
        else:
            print(f"⚠️ {name} arm not operational, check errors")

    print("=====Checking Dynamixel U2D2 Bridges Drivers   =====")


    u2d2_devices = {"Gripper": gripper_u2d2_operational, "FHRSense": fhrsense_u2d2_operational,
                    "Teleop Gripper": teleop_g_u2d2_operational, "Teleop FHRSense": teleop_c_u2d2_operational}
    
    for name, check_fn in u2d2_devices.items():
        operational = check_fn()
        if operational:
            print(f"✅ {name} U2D2 set torque mode operational")
        else:
            print(f"⚠️ {name} U2D2 not operational, check power switch and E-stop.")

    print("=====Checking Realsense Cam Drivers           =====")

    realsense_devices = {"Teleop Cam": test_teleop_realsense_capture, "Gripper Cam": test_gripper_realsense_capture, "FHRSense Cam": test_fhrsense_realsense_capture}

    for name, check_fn in realsense_devices.items():
        capture = check_fn()
        if capture:
            print(f"✅ {name} Realsense camera capture returns frames")
        else:
            print(f"⚠️ {name} Realsense camera capture failed, check connection and camera status")

    print("=====  VADER FVD Preflight Checklist Complete  =====")
