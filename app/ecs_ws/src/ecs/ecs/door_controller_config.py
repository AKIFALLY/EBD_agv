from dataclasses import dataclass
from typing import Dict, List
import yaml

@dataclass
class DoorConfig:
    door_id: int
    mr_type: str
    mr_address: str
    dm_type: str
    dm_address: str

class DoorControllerConfig:
    def __init__(self):
        self.doors: Dict[int, DoorConfig] = {}

    def set_config(self, door_list: List[str]):
        """
        由 ROS2 傳入的 string list 設定:
        每一筆格式: "1,MR,100,DM,200"
        """
        for item in door_list:
            parts = item.strip().split(',')
            if len(parts) != 5:
                raise ValueError(f"門設定格式錯誤: {item}")
            door_id = int(parts[0])
            self.doors[door_id] = DoorConfig(
                door_id,
                parts[1], parts[2],     
                parts[3], parts[4]
            )

    def load_config_yaml(self, path: str):
        """
        從 YAML 載入設定:
        YAML 檔案格式每一筆是："1,MR,100,DM,200" 這樣的字串
        """
        with open(path, 'r') as f:
            door_list = yaml.safe_load(f)
    
        self.set_config(door_list['doors'])

    def get_config(self, door_id: int) -> Dict:
        if door_id not in self.doors:
            return None
        cfg = self.doors[door_id]
        return {
            "doorId": cfg.door_id,
            "mr_type": cfg.mr_type,
            "mr_address": cfg.mr_address,
            "dm_type": cfg.dm_type,
            "dm_address": cfg.dm_address
        }

    @classmethod
    def from_list(cls, door_list: List[str]) -> "DoorControllerConfig":
        obj = cls()
        obj.set_config(door_list)
        return obj

    @classmethod
    def from_yaml(cls, path: str) -> "DoorControllerConfig":
        obj = cls()
        obj.load_config_yaml(path)
        return obj
