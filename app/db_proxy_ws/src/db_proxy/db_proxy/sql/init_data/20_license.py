"""
20. License 初始化資料
無相依性
"""

from db_proxy.models import License
from sqlmodel import select


def initialize_license(session):
    """初始化 License 資料"""
    print("🔐 初始化 License 資料...")

    default_licenses = [
        # 操作員工作站（原有 OPUI 功能）
        {
            "device_id": "ca08777c72096c51",
            "active": 1,
            "device_type": "op_station",
            "description": "操作員工作站 #1",
            "permissions": {
                "can_call_agv": True,
                "can_view_tasks": True,
                "can_create_tasks": True
            }
        },
        # HMI 終端 1 - ManualReceiveArea 測試區域（5個 Location）
        {
            "device_id": "476e01ab82a53f9e",
            "active": 1,
            "device_type": "hmi_terminal",
            "description": "ManualReceiveArea HMI",
            "permissions": {
                "locations": ["ManualReceiveArea_1", "ManualReceiveArea_2"],
                "layout": "1x2",
                "can_remove_rack": True
            }
        },
        # HMI 終端 2 -  測試用（8個 Location）
        {
            "device_id": "476e01ab82a53f9a",
            "active": 1,
            "device_type": "hmi_terminal",
            "description": "測試用 HMI",
            "permissions": {
                "locations": ["射出機2-停車位置1","射出機2-停車位置2","ManualReceiveArea_1", "ManualReceiveArea_2", "SystemReadyArea_1", "SystemEmptyRackArea_1", "房間2入口(KUKA)", "房間2出口(KUKA)"],
                "layout": "2x4",
                "can_remove_rack": True,
                "can_add_rack": True,
                "can_edit_rack": True
            }
        }
    ]

    # 檢查是否已存在相同的 device_id
    for data in default_licenses:
        device_id = data.get("device_id")
        exists = session.exec(
            select(License).where(License.device_id == device_id)
        ).first()
        if not exists:
            session.add(License(**data))

    session.commit()
    print("✅ License 資料初始化完成（包含 op_station 和 hmi_terminal 類型）")
