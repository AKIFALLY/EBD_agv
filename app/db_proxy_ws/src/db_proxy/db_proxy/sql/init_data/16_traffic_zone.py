"""
交管區初始化資料
"""

from db_proxy.models import TrafficZone
from db_proxy.sql.db_install import insert_data_if_not_exists_name_and_not_exists_id

def initialize_traffic_zones(session):
    """初始化交管區資料"""
    print("📋 初始化交管區資料...")
    
    traffic_zones_data = [
            {
                "name": "room2",
                "description": "房間2外走廊-基本交管區(points尚未定義)",
                "points": '[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1},{"x":0,"y":1}]',
                "status": "free",
                "owner_agv_id": None,
                "enable": True
            }
        ]
    
    insert_data_if_not_exists_name_and_not_exists_id(
        session, traffic_zones_data, TrafficZone
    )
    
    print(f"   ✅ 交管區資料: {len(traffic_zones_data)} 筆")
