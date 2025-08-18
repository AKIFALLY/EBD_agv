"""
03. 貨架狀態初始化資料
無相依性
"""

from db_proxy.models import RackStatus
from ..db_install import insert_data_if_not_exists_name


def initialize_rack_status(session):
    """初始化貨架狀態資料"""
    print("📦 初始化貨架狀態資料...")
    
    # 修正為運行狀態定義（與 AGVCUI 界面顯示邏輯一致）
    # AGVCUI templates/racks.html 使用這些 status_id 值
    default_rack_status = [
        {"id": 1, "name": "空閒", "description": "Rack 可供使用"},
        {"id": 2, "name": "使用中", "description": "Rack 正在執行任務"},
        {"id": 3, "name": "維護中", "description": "Rack 在維護狀態"},
        {"id": 4, "name": "故障", "description": "Rack 發生故障"},
        # 保留原本的載具相關狀態供參考（可選）
        {"id": 5, "name": "空架", "description": "全空料架未使用（載具狀態）"},
        {"id": 6, "name": "滿載", "description": "滿載料架（載具狀態）"},
        {"id": 7, "name": "部分載具", "description": "部分載具（載具狀態）"}
    ]
    
    insert_data_if_not_exists_name(session, default_rack_status, RackStatus)
    print("✅ 貨架狀態資料初始化完成")
