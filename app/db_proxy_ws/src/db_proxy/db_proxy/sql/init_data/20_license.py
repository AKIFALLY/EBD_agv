"""
20. License 初始化資料
無相依性
"""

from db_proxy.models import License
from sqlmodel import select


def initialize_license(session):
    """初始化 License 資料"""
    print("🔐 初始化 License 資料...")

    default_license = [
        {"device_id": "ca08777c72096c51", "active": 1}
    ]

    # 由於 License 沒有 name 欄位，我們需要使用不同的檢查方式
    # 檢查是否已存在相同的 device_id
    for data in default_license:
        device_id = data.get("device_id")
        exists = session.exec(
            select(License).where(License.device_id == device_id)
        ).first()
        if not exists:
            session.add(License(**data))

    session.commit()
    print("✅ License 資料初始化完成")
