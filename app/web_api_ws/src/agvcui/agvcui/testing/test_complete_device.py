#!/usr/bin/env python3
"""
測試完整設備管理功能
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
    from agvcui.db import (
        get_complete_device, create_complete_device, 
        update_complete_device, delete_complete_device
    )
except ImportError as e:
    print(f"導入錯誤: {e}")
    print("請確保所有依賴模組都已正確安裝")
    sys.exit(1)


def test_complete_device_management():
    """測試完整設備管理功能"""
    print("開始測試完整設備管理功能...")
    
    # 1. 測試創建完整設備
    print("\n1. 測試創建完整設備")
    device_data = {
        "name": "測試完整設備",
        "description": "這是一個測試的完整設備",
        "location_id": 999,
        "ports": [
            {
                "name": "TestPort01",
                "description": "測試端口01",
                "signals": [
                    {
                        "name": "TestSignal01",
                        "description": "測試信號01",
                        "value": "1",
                        "type_of_value": "boolean",
                        "dm_address": "D100"
                    },
                    {
                        "name": "TestSignal02",
                        "description": "測試信號02",
                        "value": "0",
                        "type_of_value": "boolean",
                        "dm_address": "D101"
                    }
                ]
            },
            {
                "name": "TestPort02",
                "description": "測試端口02",
                "signals": [
                    {
                        "name": "TestSignal03",
                        "description": "測試信號03",
                        "value": "100",
                        "type_of_value": "integer",
                        "dm_address": "D102"
                    }
                ]
            }
        ],
        "device_signals": [
            {
                "name": "DeviceStatus",
                "description": "設備狀態",
                "value": "running",
                "type_of_value": "string",
                "dm_address": "D200"
            }
        ]
    }
    
    try:
        new_device = create_complete_device(device_data)
        print(f"✅ 成功創建完整設備，ID: {new_device['id']}")
        print(f"   設備名稱: {new_device['name']}")
        print(f"   端口數量: {len(new_device['ports'])}")
        print(f"   設備信號數量: {len(new_device['device_signals'])}")
        
        device_id = new_device['id']
        
        # 2. 測試獲取完整設備
        print("\n2. 測試獲取完整設備")
        retrieved_device = get_complete_device(device_id)
        if retrieved_device:
            print(f"✅ 成功獲取完整設備")
            print(f"   設備名稱: {retrieved_device['name']}")
            print(f"   描述: {retrieved_device['description']}")
            print(f"   位置ID: {retrieved_device['location_id']}")
            
            print(f"   端口詳情:")
            for port in retrieved_device['ports']:
                print(f"     - {port['name']}: {port['description']}")
                for signal in port['signals']:
                    print(f"       * {signal['name']}: {signal['value']} ({signal['type_of_value']})")
            
            print(f"   設備信號:")
            for signal in retrieved_device['device_signals']:
                print(f"     - {signal['name']}: {signal['value']} ({signal['type_of_value']})")
        
        # 3. 測試更新完整設備
        print("\n3. 測試更新完整設備")
        updated_data = {
            "name": "更新後的完整設備",
            "description": "這是更新後的完整設備",
            "location_id": 888,
            "ports": [
                {
                    "name": "UpdatedPort01",
                    "description": "更新後的端口01",
                    "signals": [
                        {
                            "name": "UpdatedSignal01",
                            "description": "更新後的信號01",
                            "value": "true",
                            "type_of_value": "boolean",
                            "dm_address": "D300"
                        }
                    ]
                }
            ],
            "device_signals": [
                {
                    "name": "UpdatedDeviceStatus",
                    "description": "更新後的設備狀態",
                    "value": "updated",
                    "type_of_value": "string",
                    "dm_address": "D400"
                }
            ]
        }
        
        updated_device = update_complete_device(device_id, updated_data)
        if updated_device:
            print(f"✅ 成功更新完整設備")
            print(f"   更新後名稱: {updated_device['name']}")
            print(f"   更新後位置ID: {updated_device['location_id']}")
            print(f"   更新後端口數量: {len(updated_device['ports'])}")
            print(f"   更新後設備信號數量: {len(updated_device['device_signals'])}")
        
        # 4. 測試刪除完整設備
        print("\n4. 測試刪除完整設備")
        success = delete_complete_device(device_id)
        if success:
            print(f"✅ 成功刪除完整設備")
            
            # 驗證設備已被刪除
            deleted_device = get_complete_device(device_id)
            if not deleted_device:
                print(f"✅ 確認設備已被完全刪除")
            else:
                print(f"❌ 設備刪除失敗，仍然存在")
        
        print("\n✅ 完整設備管理功能測試完成！")
        print("💡 總結:")
        print("   - 創建完整設備：包含設備基本信息、端口和信號")
        print("   - 獲取完整設備：返回完整的設備結構")
        print("   - 更新完整設備：重新創建所有端口和信號")
        print("   - 刪除完整設備：安全刪除所有相關數據")
        
    except Exception as e:
        print(f"❌ 測試失敗: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_complete_device_management()
