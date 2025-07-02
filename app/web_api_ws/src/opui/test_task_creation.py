#!/usr/bin/env python3
"""
測試 OPUI 任務創建功能
"""

import sys
import os

# 添加 opui 模組路徑
sys.path.append(os.path.join(os.path.dirname(__file__), 'opui'))

def test_task_creation():
    """測試任務創建功能"""
    try:
        from opui.db import (
            create_task, 
            get_call_empty_work_id, 
            get_dispatch_full_work_id,
            get_default_task_status_id,
            work_all,
            task_status_all
        )
        
        print("🧪 開始測試 OPUI 任務創建功能...")
        
        # 1. 測試獲取工作類型
        print("\n1. 測試工作類型...")
        try:
            call_empty_work_id = get_call_empty_work_id()
            dispatch_full_work_id = get_dispatch_full_work_id()
            print(f"   ✅ 叫空車工作類型 ID: {call_empty_work_id}")
            print(f"   ✅ 派滿車工作類型 ID: {dispatch_full_work_id}")
        except Exception as e:
            print(f"   ❌ 獲取工作類型失敗: {e}")
            return False
        
        # 2. 測試獲取任務狀態
        print("\n2. 測試任務狀態...")
        try:
            default_status_id = get_default_task_status_id()
            print(f"   ✅ 預設任務狀態 ID: {default_status_id}")
        except Exception as e:
            print(f"   ❌ 獲取任務狀態失敗: {e}")
            return False
        
        # 3. 測試創建叫空車任務
        print("\n3. 測試創建叫空車任務...")
        try:
            call_empty_task_data = {
                "name": "測試叫空車任務",
                "description": "這是一個測試叫空車任務",
                "work_id": call_empty_work_id,
                "status_id": default_status_id,
                "priority": 1,
                "parameters": {
                    "parking_space": {"id": 1, "name": "001"},
                    "machine_id": 1,
                    "client_id": "test_client",
                    "task_type": "call_empty"
                }
            }
            
            call_empty_task = create_task(call_empty_task_data)
            print(f"   ✅ 叫空車任務創建成功: ID={call_empty_task['id']}, 名稱={call_empty_task['name']}")
            
        except Exception as e:
            print(f"   ❌ 創建叫空車任務失敗: {e}")
            return False
        
        # 4. 測試創建派滿車任務
        print("\n4. 測試創建派滿車任務...")
        try:
            dispatch_full_task_data = {
                "name": "測試派滿車任務",
                "description": "這是一個測試派滿車任務",
                "work_id": dispatch_full_work_id,
                "status_id": default_status_id,
                "priority": 2,
                "parameters": {
                    "parking_space": {"id": 2, "name": "002"},
                    "product_name": "測試產品",
                    "count": 50,
                    "rack_id": 123,
                    "room": 1,
                    "side": "left",
                    "machine_id": 1,
                    "client_id": "test_client",
                    "task_type": "dispatch_full"
                }
            }
            
            dispatch_full_task = create_task(dispatch_full_task_data)
            print(f"   ✅ 派滿車任務創建成功: ID={dispatch_full_task['id']}, 名稱={dispatch_full_task['name']}")
            
        except Exception as e:
            print(f"   ❌ 創建派滿車任務失敗: {e}")
            return False
        
        # 5. 顯示所有工作類型和任務狀態
        print("\n5. 顯示系統資料...")
        try:
            works = work_all()
            statuses = task_status_all()
            
            print(f"   📋 工作類型總數: {len(works)}")
            for work in works:
                print(f"      - ID: {work['id']}, 名稱: {work['name']}")
            
            print(f"   📊 任務狀態總數: {len(statuses)}")
            for status in statuses:
                print(f"      - ID: {status['id']}, 名稱: {status['name']}")
                
        except Exception as e:
            print(f"   ❌ 獲取系統資料失敗: {e}")
            return False
        
        print("\n🎉 所有測試通過！OPUI 任務創建功能正常運作。")
        return True
        
    except ImportError as e:
        print(f"❌ 匯入模組失敗: {e}")
        print("請確保已正確安裝相關依賴並設定環境變數。")
        return False
    except Exception as e:
        print(f"❌ 測試過程中發生未預期錯誤: {e}")
        return False


if __name__ == "__main__":
    success = test_task_creation()
    sys.exit(0 if success else 1)
