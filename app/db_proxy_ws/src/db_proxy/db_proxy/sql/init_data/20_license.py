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
        # 射出機操作員（原有 OPUI 功能）
        {
            "device_id": "ca08777c72096c51",
            "active": 1,
            "device_type": "injection_machine",
            "description": "射出機 #1 操作面板",
            "permissions": {
                "can_call_agv": True,
                "can_view_tasks": True,
                "can_create_tasks": True
            }
        },
        # HMI 終端 1 - 倉儲區區域（2個 Location）
        {
            "device_id": "476e01ab82a53f9e",
            "active": 1,
            "device_type": "hmi_terminal",
            "description": "倉儲區 HMI",
            "permissions": {
                "locations": ["LOC001", "LOC002"],
                "layout": "1x2",
                "can_remove_rack": True
            }
        },
        # HMI 終端 2 - NG區域（4個 Location）
        {
            "device_id": "hmi00000000002",
            "active": 1,
            "device_type": "hmi_terminal",
            "description": "NG區 HMI",
            "permissions": {
                "locations": ["LOC101", "LOC102", "LOC103", "LOC104"],
                "layout": "2x2",
                "can_remove_rack": True
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
    print("✅ License 資料初始化完成（包含 injection_machine 和 hmi_terminal 類型）")
