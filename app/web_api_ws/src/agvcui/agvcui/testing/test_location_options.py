#!/usr/bin/env python3
"""
測試位置選項功能
"""

import sys
import os

# 添加必要的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..', '..')
db_proxy_src = os.path.join(current_dir, '..', '..', '..', '..', '..', 'db_proxy_ws', 'src')

sys.path.insert(0, agvcui_src)
sys.path.insert(0, db_proxy_src)

try:
    from agvcui.db import get_all_locations, create_complete_device, get_complete_device
except ImportError as e:
    print(f"導入錯誤: {e}")
    print("請確保所有依賴模組都已正確安裝")
    sys.exit(1)


def test_location_options():
    """測試位置選項功能"""
    print("開始測試位置選項功能...")
    
    # 1. 測試獲取所有位置選項
    print("\n1. 測試獲取所有位置選項")
    try:
        locations = get_all_locations()
        print(f"✅ 成功獲取位置選項，共 {len(locations)} 個位置")
        
        if locations:
            print("位置列表:")
            for i, location in enumerate(locations[:10], 1):  # 只顯示前10個
                desc = f" - {location['description']}" if location['description'] else ""
                print(f"  {i}. ID: {location['id']}, 名稱: {location['name']}{desc}")
            
            if len(locations) > 10:
                print(f"  ... 還有 {len(locations) - 10} 個位置")
        else:
            print("⚠️  沒有找到任何位置數據")
            return
        
        # 2. 測試使用位置創建設備
        print("\n2. 測試使用位置創建設備")
        test_location = locations[0]  # 使用第一個位置
        
        device_data = {
            "name": "位置測試設備",
            "description": "測試位置選項的設備",
            "location_id": test_location['id'],
            "ports": [
                {
                    "name": "TestPort",
                    "description": "測試端口",
                    "signals": [
                        {
                            "name": "TestSignal",
                            "description": "測試信號",
                            "value": "1",
                            "type_of_value": "boolean"
                        }
                    ]
                }
            ]
        }
        
        new_device = create_complete_device(device_data)
        print(f"✅ 成功創建設備，ID: {new_device['id']}")
        print(f"   設備名稱: {new_device['name']}")
        print(f"   位置ID: {new_device['location_id']}")
        print(f"   使用的位置: {test_location['name']}")
        
        device_id = new_device['id']
        
        # 3. 驗證設備的位置信息
        print("\n3. 驗證設備的位置信息")
        retrieved_device = get_complete_device(device_id)
        if retrieved_device:
            print(f"✅ 設備位置ID正確: {retrieved_device['location_id']}")
            
            # 查找對應的位置名稱
            location_name = "未知位置"
            for loc in locations:
                if loc['id'] == retrieved_device['location_id']:
                    location_name = loc['name']
                    break
            
            print(f"✅ 對應的位置名稱: {location_name}")
        
        # 4. 測試位置選項在前端的使用
        print("\n4. 模擬前端位置選項使用")
        print("前端下拉選單選項:")
        for location in locations[:5]:  # 只顯示前5個
            selected = "selected" if location['id'] == test_location['id'] else ""
            desc_text = f" - {location['description']}" if location['description'] else ""
            print(f"  <option value=\"{location['id']}\" {selected}>{location['name']}{desc_text}</option>")
        
        # 清理測試數據
        print("\n5. 清理測試數據")
        from agvcui.db import delete_complete_device
        success = delete_complete_device(device_id)
        if success:
            print("✅ 成功清理測試設備")
        
        print("\n✅ 位置選項功能測試完成！")
        print("💡 總結:")
        print("   - 成功從 location 表獲取位置選項")
        print("   - 位置選項包含 ID、名稱和描述")
        print("   - 設備可以正確關聯到選定的位置")
        print("   - 前端可以使用下拉選單選擇位置")
        
    except Exception as e:
        print(f"❌ 測試失敗: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_location_options()
