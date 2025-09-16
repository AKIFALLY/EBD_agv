#!/usr/bin/env python3
"""測試 location 查詢功能"""

from flow_wcs.database_unified import db_manager

def test_location_query():
    """測試 location 查詢功能"""
    print("=== 測試 Location 查詢 ===\n")
    
    # 使用全局的資料庫管理器
    db = db_manager
    
    try:
        # 測試查詢位置
        print("1. 測試查詢所有位置:")
        locations = db.query_locations()
        print(f"   ✅ 成功查詢到 {len(locations)} 個位置")
        
        if locations:
            # 顯示第一個位置的詳細資訊
            loc = locations[0]
            print(f"\n   第一個位置詳細資訊:")
            print(f"   - ID: {loc.get('id')}")
            print(f"   - Name: {loc.get('name')}")
            print(f"   - Type: {loc.get('type')}")
            print(f"   - Room ID: {loc.get('room_id')}")
            print(f"   - X: {loc.get('x')}")
            print(f"   - Y: {loc.get('y')}")
            print(f"   - Has Rack: {loc.get('has_rack')}")
            print(f"   - Rack ID: {loc.get('rack_id')}")
            print(f"   - Status: {loc.get('status')}")
        
        # 測試查詢特定類型的位置
        print("\n2. 測試查詢特定類型位置:")
        locations_by_type = db.query_locations(type="enter_or_exit")
        print(f"   ✅ 查詢到 {len(locations_by_type)} 個 enter_or_exit 類型的位置")
        
        # 測試查詢特定房間的位置
        print("\n3. 測試查詢特定房間位置:")
        locations_by_room = db.query_locations(rooms=[1, 2])
        print(f"   ✅ 查詢到 {len(locations_by_room)} 個房間 1 和 2 的位置")
        
        # 測試查詢有架台的位置
        print("\n4. 測試查詢有架台的位置:")
        locations_with_rack = db.query_locations(has_rack=True)
        print(f"   ✅ 查詢到 {len(locations_with_rack)} 個有架台的位置")
        
        print("\n=== Location 查詢測試完成 ===")
        return True
        
    except Exception as e:
        print(f"   ❌ 錯誤: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_location_query()
    exit(0 if success else 1)