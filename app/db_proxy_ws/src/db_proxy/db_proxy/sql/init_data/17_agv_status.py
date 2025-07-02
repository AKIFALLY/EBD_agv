"""
初始化 AGV 狀態資料
"""
from db_proxy.models import AgvStatus


def init_agv_status(session):
    """初始化 AGV 狀態資料"""
    
    # 檢查是否已有資料
    existing_count = session.query(AgvStatus).count()
    if existing_count > 0:
        print(f"AGV 狀態資料已存在 ({existing_count} 筆)，跳過初始化")
        return

    # AGV 狀態資料 (參考 KUKA 機器人狀態定義)
    agv_statuses = [
        {"id": 1, "name": "離場", "description": "AGV 已離開工作區域", "color": "is-dark"},
        {"id": 2, "name": "離線", "description": "AGV 離線或無法通訊", "color": "is-danger"},
        {"id": 3, "name": "空閒", "description": "AGV 空閒，可接受新任務", "color": "is-success"},
        {"id": 4, "name": "任務中", "description": "AGV 正在執行任務", "color": "is-info"},
        {"id": 5, "name": "充電中", "description": "AGV 正在充電", "color": "is-warning"},
        {"id": 6, "name": "更新中", "description": "AGV 正在進行軟體更新", "color": "is-link"},
        {"id": 7, "name": "異常", "description": "AGV 發生異常或錯誤", "color": "is-danger"},
        {"id": 8, "name": "維護中", "description": "AGV 正在進行維護", "color": "is-warning"},
        {"id": 9, "name": "待機", "description": "AGV 待機狀態", "color": "is-light"},
        {"id": 10, "name": "初始化", "description": "AGV 正在初始化", "color": "is-info"}
    ]

    # 插入資料
    for status_data in agv_statuses:
        agv_status = AgvStatus(**status_data)
        session.add(agv_status)

    session.commit()
    print(f"已初始化 {len(agv_statuses)} 筆 AGV 狀態資料")


if __name__ == "__main__":
    from db_proxy.connection_pool_manager import ConnectionPoolManager
    
    # 初始化連線池
    connection_pool = ConnectionPoolManager()
    
    with connection_pool.get_session() as session:
        init_agv_status(session)
