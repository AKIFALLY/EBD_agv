#!/usr/bin/env python3
"""
Product CRUD 使用範例
展示如何使用新增的 product_crud 便利方法
"""

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.crud.product_crud import product_crud

# 資料庫連線 URL
DB_URL = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"

def example_product_operations():
    """展示 product_crud 的各種操作"""
    
    # 建立連線池
    pool = ConnectionPoolManager(DB_URL)
    
    try:
        with pool.get_session() as session:
            product_id = 1  # 測試產品ID
            
            print(f"📦 測試產品 {product_id} 的各種操作...")
            
            # 1. 取得製程設定ID
            process_id = product_crud.get_process_settings_id(session, product_id)
            print(f"⚙️ 製程設定ID: {process_id}")
            
            # 2. 根據名稱查詢產品
            product_name = "ABC12345"
            product = product_crud.get_product_by_name(session, product_name)
            if product:
                print(f"🔍 找到產品: {product.name} (ID: {product.id}, 尺寸: {product.size})")
            else:
                print(f"❌ 找不到產品: {product_name}")
            
            # 3. 取得產品完整資訊
            product_info = product_crud.get_product_info(session, product_id)
            if product_info:
                print(f"📋 產品完整資訊:")
                for key, value in product_info.items():
                    print(f"   {key}: {value}")
            
            # 4. 檢查產品是否存在
            exists = product_crud.is_product_exists(session, "ABC12345")
            print(f"✅ 產品 'ABC12345' 存在: {exists}")
            
            print("\n" + "="*50)
            
            # 5. 根據製程設定ID查詢產品
            process_settings_id = 1
            products_by_process = product_crud.get_products_by_process_settings_id(session, process_settings_id)
            print(f"🔧 製程設定 {process_settings_id} 的產品:")
            for product in products_by_process:
                print(f"   - {product.name} (尺寸: {product.size})")
            
            # 6. 根據尺寸查詢產品
            size = "S"
            products_by_size = product_crud.get_products_by_size(session, size)
            print(f"📏 尺寸 '{size}' 的產品:")
            for product in products_by_size:
                print(f"   - {product.name} (製程設定ID: {product.process_settings_id})")
            
            print("\n" + "="*50)
            
            # 7. 列出所有產品
            print("📋 所有產品列表:")
            all_products = product_crud.get_all(session)
            for product in all_products:
                print(f"   產品 {product.id}: {product.name}")
                print(f"      尺寸: {product.size}")
                print(f"      製程設定ID: {product.process_settings_id}")
                print(f"      建立時間: {product.created_at}")
                print()
                
    except Exception as e:
        print(f"❌ 操作失敗: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 關閉連線池
        pool.shutdown()

def example_wcs_usage():
    """展示在 WCS 中如何使用這些方法"""
    
    print("🔧 WCS Task Manager 使用範例:")
    print("""
    # 在 task_manager.py 中的使用方式：
    
    def handle_dispatch_full_task(self, task, params, session):
        rack_id = params.get("rack_id")
        room_id = params.get("room_id")
        
        # 取得 rack 上的產品
        rack = session.get(Rack, rack_id)
        if not rack or not rack.product_id:
            self.logger.error("Rack 沒有產品")
            return
            
        # 使用便利方法取得產品的製程設定ID
        product_process_id = product_crud.get_process_settings_id(session, rack.product_id)
        if not product_process_id:
            self.logger.error(f"找不到產品 {rack.product_id} 的製程設定")
            return
            
        # 取得房間的製程設定ID
        room_process_id = room_crud.get_process_settings_id(session, room_id)
        
        # 檢查製程是否匹配
        if product_process_id != room_process_id:
            self.logger.info("產品製程與房間要求不符")
            return
            
        # 取得產品詳細資訊用於記錄
        product_info = product_crud.get_product_info(session, rack.product_id)
        self.logger.info(f"派送產品: {product_info['name']} (尺寸: {product_info['size']})")
        
        # 繼續任務處理邏輯...
    
    # 其他實用場景：
    
    def validate_product_compatibility(self, product_name: str, room_id: int, session):
        # 根據產品名稱查詢產品
        product = product_crud.get_product_by_name(session, product_name)
        if not product:
            return False, "產品不存在"
            
        # 檢查製程相容性
        product_process_id = product.process_settings_id
        room_process_id = room_crud.get_process_settings_id(session, room_id)
        
        if product_process_id != room_process_id:
            return False, "製程不相容"
            
        return True, "相容"
    """)

if __name__ == "__main__":
    print("🚀 Product CRUD 功能展示")
    example_product_operations()
    example_wcs_usage()
