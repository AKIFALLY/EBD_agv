#!/usr/bin/env python3
"""
測試新的 Service 層架構
"""

import sys
import os

# 添加路徑以便導入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.services import ProductService, LocationService, MachineService, RoomService


def test_service_architecture():
    """測試新的 Service 層架構"""
    print("🧪 測試新的 Service 層架構...")
    
    # 初始化資料庫連線池
    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    
    try:
        pool = ConnectionPoolManager(db_url)
        print("✅ 資料庫連線池建立成功")
    except Exception as e:
        print(f"❌ 資料庫連線失敗: {e}")
        return False
    
    try:
        with pool.get_session() as session:
            # 測試 ProductService
            print("\n📦 測試 ProductService...")
            product_service = ProductService()
            
            # 測試取得所有產品
            products = product_service.get_all_products(session)
            print(f"   ✅ 找到 {len(products)} 個產品")
            
            if products:
                # 測試取得製程設定ID
                first_product = products[0]
                process_id = product_service.get_process_settings_id(first_product.id, session)
                print(f"   ✅ 產品 {first_product.name} 的製程設定ID: {process_id}")
                
                # 測試根據名稱查詢
                product_by_name = product_service.get_by_name(first_product.name, session)
                if product_by_name:
                    print(f"   ✅ 根據名稱找到產品: {product_by_name.name}")
                
                # 測試產品存在性檢查
                exists = product_service.is_product_exists(first_product.name, session)
                print(f"   ✅ 產品存在性檢查: {exists}")
            
            # 測試 LocationService
            print("\n📍 測試 LocationService...")
            location_service = LocationService()
            
            # 測試取得所有位置
            locations = location_service.get_all_locations(session)
            print(f"   ✅ 找到 {len(locations)} 個位置")
            
            if locations:
                # 測試根據節點ID查詢
                first_location = locations[0]
                if first_location.node_id:
                    location_by_node = location_service.get_by_node_id(first_location.node_id, session)
                    if location_by_node:
                        print(f"   ✅ 根據節點ID找到位置: {location_by_node.name}")
                
                # 測試位置可用性檢查
                is_available = location_service.is_location_available(first_location)
                print(f"   ✅ 位置可用性檢查: {is_available}")
            
            # 測試 MachineService
            print("\n🏭 測試 MachineService...")
            machine_service = MachineService()
            
            # 測試取得所有機器
            machines = machine_service.get_all_machines(session)
            print(f"   ✅ 找到 {len(machines)} 個機器")
            
            if machines:
                first_machine = machines[0]
                
                # 測試停車格可用性檢查
                is_parking_available = machine_service.is_parking_available(first_machine.id, 1, session)
                print(f"   ✅ 機器 {first_machine.name} 停車格1可用性: {is_parking_available}")
                
                # 測試機器啟用狀態
                is_enabled = machine_service.is_machine_enabled(first_machine.id, session)
                print(f"   ✅ 機器 {first_machine.name} 啟用狀態: {is_enabled}")
                
                # 測試取得可用停車格
                available_spaces = machine_service.get_available_parking_spaces(first_machine.id, session)
                print(f"   ✅ 機器 {first_machine.name} 可用停車格: {available_spaces}")
            
            # 測試 RoomService
            print("\n🏠 測試 RoomService...")
            room_service = RoomService()
            
            # 測試取得所有房間
            rooms = room_service.get_all_rooms(session)
            print(f"   ✅ 找到 {len(rooms)} 個房間")
            
            if rooms:
                first_room = rooms[0]
                
                # 測試取得製程設定ID
                process_id = room_service.get_process_settings_id(first_room.id, session)
                print(f"   ✅ 房間 {first_room.name} 的製程設定ID: {process_id}")
                
                # 測試房間啟用狀態
                is_enabled = room_service.is_room_enabled(first_room.id, session)
                print(f"   ✅ 房間 {first_room.name} 啟用狀態: {is_enabled}")
                
                # 測試取得房間位置資訊
                locations_info = room_service.get_room_locations(first_room.id, session)
                if locations_info:
                    print(f"   ✅ 房間位置資訊: 入口={locations_info.get('enter_location_id')}, 出口={locations_info.get('exit_location_id')}")
            
            print("\n🎉 所有 Service 層測試完成！")
            return True
            
    except Exception as e:
        print(f"❌ 測試過程中發生錯誤: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        # 關閉連線池
        pool.shutdown()


if __name__ == "__main__":
    print("🚀 開始測試新的 Service 層架構")
    success = test_service_architecture()
    
    if success:
        print("\n✅ 架構測試成功！新的 Service 層正常運作")
    else:
        print("\n❌ 架構測試失敗，請檢查錯誤訊息")
    
    sys.exit(0 if success else 1)
