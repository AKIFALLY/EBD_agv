#!/usr/bin/env python3
"""
Room CRUD 使用範例
展示如何使用新增的 room_crud 便利方法
"""

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.crud.room_crud import room_crud

# 資料庫連線 URL
DB_URL = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"

def example_room_operations():
    """展示 room_crud 的各種操作"""
    
    # 建立連線池
    pool = ConnectionPoolManager(DB_URL)
    
    try:
        with pool.get_session() as session:
            room_id = 1  # 測試房間ID
            
            print(f"🏠 測試房間 {room_id} 的各種操作...")
            
            # 1. 取得入口位置ID
            enter_id = room_crud.get_enter_location_id(session, room_id)
            print(f"📍 入口位置ID: {enter_id}")
            
            # 2. 取得出口位置ID
            exit_id = room_crud.get_exit_location_id(session, room_id)
            print(f"📍 出口位置ID: {exit_id}")
            
            # 3. 取得製程設定ID
            process_id = room_crud.get_process_settings_id(session, room_id)
            print(f"⚙️ 製程設定ID: {process_id}")
            
            # 4. 一次取得入口和出口位置
            locations = room_crud.get_room_locations(session, room_id)
            if locations:
                print(f"🚪 房間位置: 入口={locations['enter_location_id']}, 出口={locations['exit_location_id']}")
            else:
                print("❌ 房間不存在")
            
            # 5. 檢查房間是否啟用
            is_enabled = room_crud.is_room_enabled(session, room_id)
            print(f"✅ 房間啟用狀態: {'啟用' if is_enabled else '停用'}")
            
            print("\n" + "="*50)
            
            # 6. 列出所有房間的基本資訊
            print("📋 所有房間列表:")
            all_rooms = room_crud.get_all(session)
            for room in all_rooms:
                status = "啟用" if room.enable == 1 else "停用"
                print(f"   房間 {room.id}: {room.name} ({status})")
                print(f"      入口位置ID: {room.enter_location_id}")
                print(f"      出口位置ID: {room.exit_location_id}")
                print(f"      製程設定ID: {room.process_settings_id}")
                print()
                
    except Exception as e:
        print(f"❌ 操作失敗: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 關閉連線池
        pool.shutdown()

def example_task_manager_usage():
    """展示在 task_manager 中如何使用這些方法"""
    
    print("🔧 Task Manager 使用範例:")
    print("""
    # 在 task_manager.py 中的使用方式：
    
    def handle_dispatch_full_task(self, task, params, session):
        room_id = params.get("room_id")
        
        # 快速取得房間的入口和出口位置
        locations = room_crud.get_room_locations(session, room_id)
        if not locations:
            self.logger.error(f"房間 {room_id} 不存在")
            return
            
        enter_location_id = locations["enter_location_id"]
        exit_location_id = locations["exit_location_id"]
        
        # 檢查房間是否啟用
        if not room_crud.is_room_enabled(session, room_id):
            self.logger.warning(f"房間 {room_id} 已停用")
            return
            
        # 取得製程設定ID進行比對
        room_process_id = room_crud.get_process_settings_id(session, room_id)
        
        # 繼續任務處理邏輯...
    """)

if __name__ == "__main__":
    print("🚀 Room CRUD 功能展示")
    example_room_operations()
    example_task_manager_usage()
