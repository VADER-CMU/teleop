import yaml
from pathlib import Path


class VADERTeleopConfigReader:
    def __init__(self):
        base_path = Path(__file__).parent.resolve()
        self.config_path: str = "../configs/vader_teleop_config.yaml"
        self.config_path = base_path / self.config_path
        print(self.config_path)
        with open(self.config_path, 'r') as file:
            self.config = yaml.safe_load(file)

    @staticmethod
    def _get(config, *keys):
        value = config
        for key in keys:
            value = value.get(key)
            if value is None:
                raise KeyError(f"Missing key: {'.'.join(keys)}")
        return value

    def set_joint_offsets_gripper(self, offsets):
        self.config['teleop_right']['joint_offsets'] = offsets
        with open(self.config_path, 'w') as file:
            yaml.dump(self.config, file)

    def set_joint_offsets_fhrsense(self, offsets):
        self.config['teleop_left']['joint_offsets'] = offsets
        with open(self.config_path, 'w') as file:
            yaml.dump(self.config, file)

    def get_teleop_right_port(self) -> str:
        return self._get(self.config, "teleop_right", "port")

    def get_teleop_right_ids(self) -> list:
        return self._get(self.config, "teleop_right", "ids")

    def get_teleop_right_offsets(self) -> list:
        return self._get(self.config, "teleop_right", "joint_offsets")

    def get_teleop_right_signs(self) -> list:
        return self._get(self.config, "teleop_right", "joint_signs")

    def get_teleop_right_config(self) -> list:
        return self._get(self.config, "teleop_right", "gripper_config")

    def get_teleop_left_port(self) -> str:
        return self._get(self.config, "teleop_left", "port")

    def get_teleop_left_ids(self) -> list:
        return self._get(self.config, "teleop_left", "ids")

    def get_teleop_left_offsets(self) -> list:
        return self._get(self.config, "teleop_left", "joint_offsets")

    def get_teleop_left_signs(self) -> list:
        return self._get(self.config, "teleop_left", "joint_signs")

    def get_teleop_left_config(self) -> list:
        return self._get(self.config, "teleop_left", "gripper_config")

    def get_gripper_port(self) -> str:
        return self._get(self.config, "gripper", "port")

    def get_gripper_arm_ip(self) -> str:
        return self._get(self.config, "gripper", "arm_ip")

    def get_gripper_ids(self) -> list:
        return self._get(self.config, "gripper", "ids")

    def get_fhrsense_port(self) -> str:
        return self._get(self.config, "fhrsense", "port")

    def get_fhrsense_arm_ip(self) -> str:
        return self._get(self.config, "fhrsense", "arm_ip")

    def get_fhrsense_ids(self) -> list:
        return self._get(self.config, "fhrsense", "ids")

    def get_fhrsense_tool_ranges(self) -> list:
        return self._get(self.config, "fhrsense", "tool_ranges")
    
    def get_fhrsense_baudrate(self) -> int:
        return self._get(self.config, "fhrsense", "baudrate")

    def get_gripper_reset_joints(self) -> list:
        return self._get(self.config, "gripper", "reset_joints")

    def get_fhrsense_reset_joints(self) -> list:
        return self._get(self.config, "fhrsense", "reset_joints")

    def get_teleop_leftamera_serial(self) -> str:
        return self._get(self.config, "camera", "teleop_serial")

    def get_gripper_camera_serial(self) -> str:
        return self._get(self.config, "camera", "gripper_serial")

    def get_fhrsense_camera_serial(self) -> str:
        return self._get(self.config, "camera", "fhrsense_serial")


if __name__ == "__main__":
    reader = VADERTeleopConfigReader()
    print("Teleop G Port:", reader.get_teleop_rightripper_port())
    print("Teleop C Port:", reader.get_teleop_fhrsense_port())
    print("Teleop G IDs:", reader.get_teleop_rightripper_ids())
    print("Teleop C IDs:", reader.get_teleop_fhrsense_ids())
    print("Teleop G Offsets:", reader.get_teleop_rightripper_offsets())
    print("Teleop C Offsets:", reader.get_teleop_fhrsense_offsets())
    print("Teleop G Gripper Config:", reader.get_teleop_rightripper_config())
    print("Teleop C Gripper Config:", reader.get_teleop_fhrsense_config())
    print("Gripper Port:", reader.get_gripper_port())
    print("Gripper Arm IP:", reader.get_gripper_arm_ip())
    print("Gripper IDs:", reader.get_gripper_ids())
    print("FHRSense Port:", reader.get_fhrsense_port())
    print("FHRSense Arm IP:", reader.get_fhrsense_arm_ip())
    print("FHRSense ID:", reader.get_fhrsense_ids())
    print("Teleop Camera Serial:", reader.get_teleop_left_camera_serial())
    print("Gripper Camera Serial:", reader.get_gripper_camera_serial())
    print("FHRSense Camera Serial:", reader.get_fhrsense_camera_serial())
