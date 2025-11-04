import yaml
from pathlib import Path

class VADERTeleopConfigReader:
    def __init__(self):
        base_path = Path(__file__).parent.resolve()
        config_path: str = "../configs/vader_teleop_config.yaml"
        self.config_path = base_path / config_path
        print(self.config_path)
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

    @staticmethod
    def _get(config, *keys):
        value = config
        for key in keys:
            value = value.get(key)
            if value is None:
                raise KeyError(f"Missing key: {'.'.join(keys)}")
        return value

    def get_teleop_port(self, arm: str) -> str:
        """arm should be 'G' or 'C'."""
        key = f"teleop_{arm}"
        return self._get(self.config, key, "port")

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

if __name__ == "__main__":
    reader = VADERTeleopConfigReader()
    print("Teleop G Port:", reader.get_teleop_port("G"))
    print("Teleop C Port:", reader.get_teleop_port("C"))
    print("Gripper Port:", reader.get_gripper_port())
    print("Gripper Arm IP:", reader.get_gripper_arm_ip())
    print("Gripper IDs:", reader.get_gripper_ids())
    print("Cutter Port:", reader.get_cutter_port())
    print("Cutter Arm IP:", reader.get_cutter_arm_ip())
    print("Cutter ID:", reader.get_cutter_id())