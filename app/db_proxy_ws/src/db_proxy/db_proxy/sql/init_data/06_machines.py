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
        {"id": 1, "parking_space_1": 15, "parking_space_2": 14,
            "workspace_1": [101, 102, 103], "workspace_2": [104, 105, 106],
            "name": "射出機1", "description": "射出機1", "enable": 1},
        {"id": 2, "parking_space_1": 25, "parking_space_2": 23,
            "workspace_1": [201, 202, 203], "workspace_2": [204, 205, 206],
            "name": "射出機2", "description": "射出機2", "enable": 1},
        {"id": 3, "parking_space_1": 46, "parking_space_2": 44,
            "workspace_1": [301, 302, 303], "workspace_2": [304, 305, 306],
            "name": "射出機3", "description": "射出機3", "enable": 0},
        {"id": 4, "parking_space_1": 47, "parking_space_2": 45,
            "workspace_1": [401, 402, 403], "workspace_2": [404, 405, 406],
            "name": "射出機4", "description": "射出機4", "enable": 0},
    ]

    insert_data_if_not_exists_name(session, default_machines, Machine)
    print("✅ 機器資料初始化完成")
