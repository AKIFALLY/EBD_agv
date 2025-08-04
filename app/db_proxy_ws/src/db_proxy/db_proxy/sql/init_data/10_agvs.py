"""
10. AGV 初始化資料
無相依性，但需要在貨架之前載入
"""

from db_proxy.models import AGV
from ..db_install import insert_data_if_not_exists_name_and_not_exists_id


def initialize_agvs(session):
    """初始化 AGV 資料"""
    print("🚗 初始化 AGV 資料...")
    
    default_agv = [
        {"name": "cargo02", "model": "Cargo", "x": 0.0, "y": 0.0,
            "heading": 0.0, "description": "走廊AGV(暫時規劃僅負責房間2)"},
        {"name": "loader02", "model": "Loader", "x": 0.0, "y": 0.0,
            "heading": 0.0, "description": "房間2Loader(取入口傳送箱、清洗、泡藥、放預烘)"},
        {"name": "unloader02", "model": "Unloader", "x": 0.0, "y": 0.0,
            "heading": 0.0, "description": "房間2Unloader(取預烘、烤箱、放出口傳送箱)"},
        {"id": 8506941, "name": "KUKA001", "model": "KUKA400i", "x": 3116,
            "y": 1852, "heading": 0.0, "description": "在房間外負責料架搬運"},
        {"id": 8506995, "name": "KUKA002", "model": "KUKA400i", "x": 2860,
            "y": 1680, "heading": 0.0, "description": "在房間外負責料架搬運"},
        {"id": 123, "name": "KUKA003", "model": "KUKA400i", "x": 0.0,
            "y": 0.0, "heading": 0.0, "description": "(SimCar)在房間外負責料架搬運"}
    ]
    
    insert_data_if_not_exists_name_and_not_exists_id(session, default_agv, AGV)
    print("✅ AGV 資料初始化完成")
