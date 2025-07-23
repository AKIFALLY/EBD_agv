"""
03. 貨架狀態初始化資料
無相依性
"""

from db_proxy.models import RackStatus
from ..db_install import insert_data_if_not_exists_name


def initialize_rack_status(session):
    """初始化貨架狀態資料"""
    print("📦 初始化貨架狀態資料...")
    
    default_rack_status = [
        {"id": 1, "name": "空架", "description": "全空料架未使用"},
        {"id": 2, "name": "滿料架-32", "description": "全滿料架(32格)"},
        {"id": 3, "name": "滿料架-16", "description": "全滿料架(16格)"},
        {"id": 4, "name": "未滿架-32", "description": "半滿料架(32格)"},
        {"id": 5, "name": "未滿架-16", "description": "半滿料架(16格)"},
        {"id": 6, "name": "未滿料-無carrier", "description": "未滿料-但房間已無carrier"},
        {"id": 7, "name": "NG料架", "description": "NG料架-等待回收"}
    ]
    
    insert_data_if_not_exists_name(session, default_rack_status, RackStatus)
    print("✅ 貨架狀態資料初始化完成")
