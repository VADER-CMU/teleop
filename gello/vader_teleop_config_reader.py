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
        self.config['teleop_G']['joint_offsets'] = offsets
        with open(self.config_path, 'w') as file:
            yaml.dump(self.config, file)
        

    def set_joint_offsets_cutter(self, offsets):
        self.config['teleop_C']['joint_offsets'] = offsets
        with open(self.config_path, 'w') as file:
            yaml.dump(self.config, file)

    def get_teleop_gripper_port(self) -> str:
        return self._get(self.config, "teleop_G", "port")
    
    def get_teleop_gripper_ids(self) -> list:
        return self._get(self.config, "teleop_G", "ids")
    
    def get_teleop_gripper_offsets(self) -> list:
        return self._get(self.config, "teleop_G", "joint_offsets")

    def get_teleop_gripper_signs(self) -> list:
        return self._get(self.config, "teleop_G", "joint_signs")
    
    def get_teleop_gripper_config(self) -> list:
        return self._get(self.config, "teleop_G", "gripper_config")
    
    def get_teleop_cutter_port(self) -> str:
        return self._get(self.config, "teleop_C", "port")
    
    def get_teleop_cutter_ids(self) -> list:
        return self._get(self.config, "teleop_C", "ids")
    
    def get_teleop_cutter_offsets(self) -> list:
        return self._get(self.config, "teleop_C", "joint_offsets")

    def get_teleop_cutter_signs(self) -> list:
        return self._get(self.config, "teleop_C", "joint_signs")
    
    def get_teleop_cutter_config(self) -> list:
        return self._get(self.config, "teleop_C", "gripper_config")

    def get_gripper_port(self) -> str:
        return self._get(self.config, "gripper", "port")

    def get_gripper_arm_ip(self) -> str:
        return self._get(self.config, "gripper", "arm_ip")

    def get_gripper_ids(self) -> list:
        return self._get(self.config, "gripper", "ids")

    def get_cutter_port(self) -> str:
        return self._get(self.config, "cutter", "port")

    def get_cutter_arm_ip(self) -> str:
        return self._get(self.config, "cutter", "arm_ip")

    def get_cutter_id(self) -> int:
        return self._get(self.config, "cutter", "id")
    
    def get_gripper_reset_joints(self) -> list:
        return self._get(self.config, "gripper", "reset_joints")
    
    def get_cutter_reset_joints(self) -> list:
        return self._get(self.config, "cutter", "reset_joints")

    def get_camera_serial(self) -> str:
        return self._get(self.config, "camera", "serial")

if __name__ == "__main__":
    reader = VADERTeleopConfigReader()
    print("Teleop G Port:", reader.get_teleop_gripper_port())
    print("Teleop C Port:", reader.get_teleop_cutter_port())
    print("Teleop G IDs:", reader.get_teleop_gripper_ids())
    print("Teleop C IDs:", reader.get_teleop_cutter_ids())
    print("Teleop G Offsets:", reader.get_teleop_gripper_offsets())
    print("Teleop C Offsets:", reader.get_teleop_cutter_offsets())
    print("Teleop G Gripper Config:", reader.get_teleop_gripper_config())
    print("Teleop C Gripper Config:", reader.get_teleop_cutter_config())
    print("Gripper Port:", reader.get_gripper_port())
    print("Gripper Arm IP:", reader.get_gripper_arm_ip())
    print("Gripper IDs:", reader.get_gripper_ids())
    print("Cutter Port:", reader.get_cutter_port())
    print("Cutter Arm IP:", reader.get_cutter_arm_ip())
    print("Cutter ID:", reader.get_cutter_id())
    print("Camera Serial:", reader.get_camera_serial())