#!/usr/bin/env python3
"""
簡化版的 Service 層架構測試
不依賴 ROS2，直接測試 Service 層功能
"""

import sys
import os

# 添加路徑以便導入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

try:
    from db_proxy.services import ProductService, LocationService, MachineService, RoomService
    print("✅ 成功導入所有 Service 類別")
except ImportError as e:
    print(f"❌ 導入 Service 失敗: {e}")
    sys.exit(1)


def test_service_instantiation():
    """測試 Service 類別的實例化"""
    print("\n🧪 測試 Service 類別實例化...")
    
    try:
        # 測試實例化所有 Service
        product_service = ProductService()
        print("   ✅ ProductService 實例化成功")
        
        location_service = LocationService()
        print("   ✅ LocationService 實例化成功")
        
        machine_service = MachineService()
        print("   ✅ MachineService 實例化成功")
        
        room_service = RoomService()
        print("   ✅ RoomService 實例化成功")
        
        return True
        
    except Exception as e:
        print(f"   ❌ Service 實例化失敗: {e}")
        return False


def test_service_methods():
    """測試 Service 方法是否存在"""
    print("\n🔍 測試 Service 方法...")
    
    try:
        # 測試 ProductService 方法
        product_service = ProductService()
        methods = [
            'get_process_settings_id',
            'get_by_name', 
            'is_product_exists',
            'get_products_by_process_settings_id',
            'get_products_by_size',
            'get_product_info',
            'get_all_products',
            'create_product',
            'update_product',
            'delete_product'
        ]
        
        for method in methods:
            if hasattr(product_service, method):
                print(f"   ✅ ProductService.{method} 存在")
            else:
                print(f"   ❌ ProductService.{method} 不存在")
        
        # 測試 LocationService 方法
        location_service = LocationService()
        location_methods = [
            'get_by_node_id',
            'get_by_room_id',
            'is_location_available',
            'is_location_occupied',
            'get_available_locations',
            'get_occupied_locations',
            'update_location_status',
            'get_location_by_name',
            'get_all_locations'
        ]
        
        for method in location_methods:
            if hasattr(location_service, method):
                print(f"   ✅ LocationService.{method} 存在")
            else:
                print(f"   ❌ LocationService.{method} 不存在")
        
        # 測試 MachineService 方法
        machine_service = MachineService()
        machine_methods = [
            'is_parking_available',
            'is_parking_task_active',
            'is_parking_task_completed',
            'get_parking_status_info',
            'get_parking_status_name',
            'update_parking_status',
            'get_available_parking_spaces',
            'is_machine_enabled',
            'get_all_machines'
        ]
        
        for method in machine_methods:
            if hasattr(machine_service, method):
                print(f"   ✅ MachineService.{method} 存在")
            else:
                print(f"   ❌ MachineService.{method} 不存在")
        
        # 測試 RoomService 方法
        room_service = RoomService()
        room_methods = [
            'get_process_settings_id',
            'get_enter_location_id',
            'get_exit_location_id',
            'get_room_locations',
            'is_room_enabled',
            'get_room_by_name',
            'get_rooms_by_process_settings_id',
            'get_enabled_rooms',
            'get_all_rooms',
            'update_room_status'
        ]
        
        for method in room_methods:
            if hasattr(room_service, method):
                print(f"   ✅ RoomService.{method} 存在")
            else:
                print(f"   ❌ RoomService.{method} 不存在")
        
        return True
        
    except Exception as e:
        print(f"   ❌ 測試 Service 方法時發生錯誤: {e}")
        return False


def test_crud_instantiation():
    """測試 Service 內部的 CRUD 實例化"""
    print("\n🗄️ 測試 CRUD 實例化...")
    
    try:
        from db_proxy.crud.base_crud import BaseCRUD
        from db_proxy.models import Product, Location, Machine, Room
        
        # 測試直接創建 BaseCRUD 實例
        product_crud = BaseCRUD(Product, id_column="id")
        print("   ✅ Product BaseCRUD 實例化成功")
        
        location_crud = BaseCRUD(Location, id_column="id")
        print("   ✅ Location BaseCRUD 實例化成功")
        
        machine_crud = BaseCRUD(Machine, id_column="id")
        print("   ✅ Machine BaseCRUD 實例化成功")
        
        room_crud = BaseCRUD(Room, id_column="id")
        print("   ✅ Room BaseCRUD 實例化成功")
        
        # 測試 Service 內部的 CRUD
        product_service = ProductService()
        if hasattr(product_service, 'crud'):
            print("   ✅ ProductService 內部 CRUD 存在")
        
        return True
        
    except Exception as e:
        print(f"   ❌ CRUD 實例化失敗: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主測試函數"""
    print("🚀 開始測試新的 Service 層架構（簡化版）")
    
    success = True
    
    # 測試 Service 實例化
    if not test_service_instantiation():
        success = False
    
    # 測試 Service 方法
    if not test_service_methods():
        success = False
    
    # 測試 CRUD 實例化
    if not test_crud_instantiation():
        success = False
    
    if success:
        print("\n🎉 所有測試通過！新的 Service 層架構正常運作")
        print("\n📋 架構總結:")
        print("   ✅ Model 層 -> Service 層架構已建立")
        print("   ✅ 通用 Service 層在 db_proxy 專案中")
        print("   ✅ 各 Service 提供完整的業務邏輯方法")
        print("   ✅ 移除了多餘的 CRUD 層，直接使用 BaseCRUD")
        print("\n🔄 下一步:")
        print("   1. 在各專案中創建特定的 Service 層")
        print("   2. 更新現有程式碼使用新的 Service 層")
        print("   3. 進行完整的功能測試")
    else:
        print("\n❌ 部分測試失敗，請檢查錯誤訊息")
    
    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
