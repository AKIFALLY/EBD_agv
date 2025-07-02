"""
06. 機器初始化資料
依賴：節點
"""

from db_proxy.models import Machine
from ..db_install import insert_data_if_not_exists_name


def initialize_machines(session):
    """初始化機器資料"""
    print("🏭 初始化機器資料...")

    default_machines = [
        {"id": 1, "parking_space_1": 95, "parking_space_2": 96,
            "name": "射出機1", "description": "射出機1", "enable": 1},
        {"id": 2, "parking_space_1": 97, "parking_space_2": 98,
            "name": "射出機2", "description": "射出機2", "enable": 1},
        {"id": 3, "parking_space_1": 1005, "parking_space_2": 1006,
            "name": "射出機3", "description": "射出機3", "enable": 0},
        {"id": 4, "parking_space_1": 1007, "parking_space_2": 1008,
            "name": "射出機4", "description": "射出機4", "enable": 0},
    ]

    insert_data_if_not_exists_name(session, default_machines, Machine)
    print("✅ 機器資料初始化完成")
