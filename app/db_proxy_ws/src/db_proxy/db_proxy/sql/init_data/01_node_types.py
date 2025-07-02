"""
01. 節點類型初始化資料
無相依性，最先載入
"""

from db_proxy.models import NodeType
from ..db_install import insert_data_if_not_exists_name_and_not_exists_id


def initialize_node_types(session):
    """初始化節點類型資料"""
    print("📍 初始化節點類型資料...")
    
    default_node_types = [
        {"id": 4, "name": "充電入口點",
            "description": "KUKA充電入口點資訊由地圖檔中該點含functionList且functionType=4的Node"},
        {"id": 10, "name": "充電站點",
            "description": "KUKA充電站點資訊由地圖檔中該點含functionList且functionType=10的Node"},
        {"id": 2, "name": "貨架點",
            "description": "KUKA貨架點資訊由地圖檔中該點含functionList且functionType=2的Node"},
        {"id": 6, "name": "避讓點",
            "description": "KUKA避讓點資訊由地圖檔中該點含functionList且functionType=6的Node"},
    ]
    
    insert_data_if_not_exists_name_and_not_exists_id(session, default_node_types, NodeType)
    print("✅ 節點類型資料初始化完成")
